#!/usr/bin/env python3
"""Origin-anchored GT-vs-SLAM error from two TUM files.

The forced-goal validation frames are translation-only (the rover's spawn yaw in
world ~ slam-frame yaw ~ 0; verified on the straight out-leg where world and slam
deltas coincide). So a *global* Umeyama/yaw best-fit (compare_tum_trajectories
--align yaw) is the wrong tool: it smears a run that is healthy for the first half
and diverges in the second across the whole trajectory, inflating the early error
and hiding the discrete jump. This anchors on the first matched pose (pure
translation, optional small yaw fit on an early window) and reports the HONEST
error-vs-time profile + the first-jump time -- the signal that matters for SLAM
stability (did it stay bounded, or step-jump?).

Usage: slam_anchored_error.py --reference gt.tum --estimate slam_base.tum
         [--max-dt 0.05] [--jump-thresh 2.0] [--fit-yaw-window 30]
"""
import argparse, bisect, math, sys


def load(p):
    rows = []
    for ln in open(p):
        a = ln.split()
        if len(a) >= 3:
            try:
                rows.append((float(a[0]), float(a[1]), float(a[2])))
            except ValueError:
                pass
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reference", required=True)   # GT base-link (world frame)
    ap.add_argument("--estimate", required=True)    # SLAM, already in base frame
    ap.add_argument("--max-dt", type=float, default=0.05)
    ap.add_argument("--jump-thresh", type=float, default=2.0)   # m: diverged
    ap.add_argument("--jump-rate", type=float, default=1.0)     # m/s: a step jump
    ap.add_argument("--fit-yaw-window", type=float, default=30.0)  # s of early data to fit a yaw
    ap.add_argument("--max-speed", type=float, default=2.0)  # m/s: above = glitch sample, dropped
    args = ap.parse_args()

    gt = load(args.reference)
    sl = load(args.estimate)
    if not gt or not sl:
        print("ANCHORED VERDICT FAIL(no-data)")
        return 1
    gtt = [r[0] for r in gt]

    def nearest(t):
        i = bisect.bisect_left(gtt, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(gt)]
        return min(cands, key=lambda j: abs(gtt[j] - t)) if cands else None

    pairs = []  # (t, sx, sy, gx, gy)
    for t, x, y in sl:
        j = nearest(t)
        if j is not None and abs(gtt[j] - t) <= args.max_dt:
            pairs.append((t, x, y, gt[j][1], gt[j][2]))
    if len(pairs) < 5:
        print(f"ANCHORED VERDICT FAIL(too-few-matches={len(pairs)})")
        return 1

    # Speed-gate: drop physically-impossible glitch samples (a single corrupt GT
    # or SLAM pose pairing produces 100s-of-m/s "motion" that otherwise inflates
    # max/rate by 10x). The rover tops out ~0.2 m/s; allow generous headroom.
    dropped = 0
    clean = [pairs[0]]
    for k in range(1, len(pairs)):
        dt = pairs[k][0] - clean[-1][0]
        if dt <= 0:
            continue
        # speed in the GT (truth) stream — truth can't teleport
        v = math.hypot(pairs[k][3] - clean[-1][3], pairs[k][4] - clean[-1][4]) / dt
        if v > args.max_speed:
            dropped += 1
            continue
        clean.append(pairs[k])
    if dropped:
        print(f"speed-gate: dropped {dropped} glitch samples (>{args.max_speed} m/s)")
    pairs = clean

    t0, sx0, sy0, gx0, gy0 = pairs[0]
    # translation offset from first pose; small yaw from an early window (least-squares)
    win = [p for p in pairs if p[0] - t0 <= args.fit_yaw_window]
    yaw = 0.0
    if len(win) >= 5:
        # find yaw R minimizing |R*(s-s0) - (g-g0)| over the early window
        sxx = syy = sxy = syx = 0.0
        for t, sx, sy, gx, gy in win:
            a, b = sx - sx0, sy - sy0
            c, d = gx - gx0, gy - gy0
            sxx += a * c + b * d
            sxy += a * d - b * c
        yaw = math.atan2(sxy, sxx)
    cy, sy_ = math.cos(yaw), math.sin(yaw)

    def world_of(sx, sy):
        a, b = sx - sx0, sy - sy0
        return (cy * a - sy_ * b + gx0, sy_ * a + cy * b + gy0)

    prof = []
    for t, sx, sy, gx, gy in pairs:
        wx, wy = world_of(sx, sy)
        prof.append((t - t0, math.hypot(wx - gx, wy - gy)))

    n = len(prof)
    mx = max(e for _, e in prof)
    final = prof[-1][1]
    mean = sum(e for _, e in prof) / n
    # first sample where error crosses jump-thresh, and max instantaneous rate
    first_cross = next((tt for tt, e in prof if e > args.jump_thresh), None)
    maxrate = 0.0
    rate_t = None
    for i in range(1, n):
        dt = prof[i][0] - prof[i - 1][0]
        if dt > 0:
            r = abs(prof[i][1] - prof[i - 1][1]) / dt
            if r > maxrate:
                maxrate, rate_t = r, prof[i][0]

    print(f"anchored matched={n} yaw_fit={math.degrees(yaw):.2f}deg "
          f"offset=({gx0 - sx0:.2f},{gy0 - sy0:.2f})")
    for k in range(0, n, max(1, n // 12)):
        print(f"  t+{prof[k][0]:6.1f}s  err={prof[k][1]:.3f} m")
    print(f"anchored final={final:.3f}  max={mx:.3f}  mean={mean:.3f}  "
          f"first_cross_{args.jump_thresh}m={first_cross}  max_rate={maxrate:.2f}m/s@{rate_t}")

    # Stability verdict: a divergence is a LARGE error (the watchdog signal), not a
    # transient fast-rate blip. A jump only counts if it actually leads to a large
    # error (rate high AND max large) -- otherwise a 1.2 m/s sample reaching 0.4 m
    # is just noise. max error is the authoritative gate.
    diverged = mx > args.jump_thresh
    jump = maxrate > args.jump_rate and mx > args.jump_thresh
    verdict = "PASS(stability)" if not diverged else "FAIL(divergence)"
    print(f"ANCHORED VERDICT {verdict}: max={mx:.2f} final={final:.2f} "
          f"max_rate={maxrate:.2f}m/s jump_to_divergence={jump} diverged={diverged}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
