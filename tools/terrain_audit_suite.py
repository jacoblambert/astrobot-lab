#!/usr/bin/env python3
"""Run a terrain robustness audit suite and write a C1-C4 scorecard to the archive.

A suite (tools/audit_suites/<name>.toml) lists captures x pose profiles; checks
(tools/audit_suites/checks.toml unless the suite names another file) map report
metrics to the plan thresholds. Jobs run in parallel worker processes, one
single-threaded replay each.

  tools/uv_python.sh tools/terrain_audit_suite.py quick
  tools/uv_python.sh tools/terrain_audit_suite.py full --reuse <prior suite run> --baseline <scorecard.json>

--reuse scores the cached layers of a prior suite run when the replay key
(source, profile, every, limit, replay code) is unchanged, so scorer / gap-policy
changes re-score in seconds. Exit status: 0 all checks pass, 2 a check failed
(the scorecard is still written), 1 a job crashed.

A suite with an [rtk] table (e.g. c3.toml) instead runs audit_rtk_robustness.py jobs in
the dev container (docker exec), `parallel` at a time on distinct ROS domains, building
each recorder's shared input once when no base is given. Its report profiles are
"sup:<profile>" (navigation-mode supervisor) and "bare:<profile>" (GLIM only).
"""

import os

# one replay per core: keep native and BLAS code single-threaded in every worker
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse  # noqa: E402
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed  # noqa: E402
import fnmatch  # noqa: E402
import json  # noqa: E402
import multiprocessing  # noqa: E402
import re  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402
from pathlib import Path  # noqa: E402

import toml  # noqa: E402

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run  # noqa: E402

SUITES = Path(__file__).resolve().parent / "audit_suites"
OPS = {">=": lambda v, t: v >= t, "<=": lambda v, t: v <= t, "==": lambda v, t: v == t}
REDUCE = {"min": min, "max": max, "sum": sum}


def load_suite(name):
    path = Path(name) if name.endswith(".toml") else SUITES / f"{name}.toml"
    suite = toml.loads(path.read_text())
    checks_path = path.parent / suite["suite"].get("checks", "checks.toml")
    suite["checks"] = toml.loads(checks_path.read_text())["checks"]
    suite["path"], suite["checks_path"] = str(path), str(checks_path)
    return suite


def resolve(archive, prefix):
    matches = sorted(archive.glob(prefix + "*"))
    if len(matches) != 1:
        raise ValueError(f"{prefix}: {len(matches)} archive matches")
    return matches[0]


def slug(source, profile):
    return Path(source).name[:17] + "__" + re.sub(r"[^A-Za-z0-9.]+", "_", profile)


def expand(suite, archive):
    every, limit = suite["suite"].get("every", 1), suite["suite"].get("limit")
    jobs = []
    for group in suite["jobs"]:
        for prefix in group["sources"]:
            source = resolve(archive, prefix)
            for profile in group["profiles"]:
                jobs.append(dict(source=str(source), profile=profile, every=group.get("every", every),
                                 limit=group.get("limit", limit), slug=slug(source, profile)))
    if len({j["slug"] for j in jobs}) != len(jobs):
        raise ValueError("duplicate suite job")
    return jobs


def _work(job, archive, out, cache_from, save_layers):
    import audit_terrain_robustness as audit

    out = Path(out)
    out.mkdir()
    try:
        report = audit.run(job["source"], job["profile"], archive, out, every=job["every"], limit=job["limit"],
                           cache_from=cache_from, save_layers=save_layers)
        return dict(job, ok=True, wall_s=report["wall_s"], cached=report["layers_cached"])
    except BaseException as exc:  # recorded, the suite continues with the other jobs
        (out / "failure.json").write_text(json.dumps(dict(error=repr(exc), trace=traceback.format_exc())))
        return dict(job, ok=False, error=repr(exc))


CONTAINER_SETUP = ("source /etc/ros_setup.sh && source /workspace/astrobot-lab/astrobot-lab/install/setup.bash"
                   " && cd /workspace/astrobot-lab && ")


def container_run(container, args, log):
    """One audit_rtk_robustness.py invocation; returns its archive run directory (container path)."""
    command = CONTAINER_SETUP + "python3 tools/audit_rtk_robustness.py " + " ".join(args)
    with open(log, "w") as out:
        rc = subprocess.run(["docker", "exec", "-u", f"{os.getuid()}:{os.getgid()}", container, "bash", "-lc", command],
                            stdout=out, stderr=subprocess.STDOUT).returncode
    first = Path(log).read_text().splitlines()[:1]
    run = first[0].split(" ", 1)[1] if first and first[0].startswith("Run: ") else None
    return rc, run


def run_rtk(suite, archive, run):
    """C3 jobs in the container. Archive paths: /archive inside, `archive` on the host."""
    cfg = suite["rtk"]
    host = lambda p: Path(str(p).replace("/archive", str(archive), 1))  # noqa: E731
    inside = lambda p: "/archive/" + Path(p).name  # noqa: E731
    recorder = inside(resolve(archive, cfg["recorder"]))
    manifest = inside(resolve(archive, cfg["terrain_manifest"])) + "/manifest.json"
    container, logs = cfg.get("container", "astrobot-selected-dev"), run / "logs"
    logs.mkdir()
    bases = {mode: inside(resolve(archive, prefix)) for mode, prefix in cfg.get("bases", {}).items()}
    jobs = [dict(mode="sup" if group.get("supervised", True) else "bare", profile=profile,
                 rate=group.get("rate", cfg.get("rate", 2)))
            for group in cfg["jobs"] for profile in group["profiles"]]
    sigma = cfg.get("rtk_sigma")  # [S0, K]: base fixes rescaled to sigma(d) = S0 + K*d; absent = as recorded
    for mode, base in bases.items():
        built = json.loads((host(base) / "base.json").read_text()).get("rtk_sigma")
        if (built and [built["sigma0"], built["sigma_per_m"]] or None) != (list(sigma) if sigma else None):
            raise ValueError(f"base {mode} was built with rtk_sigma {built}, suite asks {sigma}: drop it to rebuild")
    for mode in sorted({j["mode"] for j in jobs} - set(bases)):
        args = [recorder, "--make-base"] + (["--no-supervisor"] if mode == "bare" else [])
        args += ["--rtk-sigma", ",".join(map(str, sigma))] if sigma else []
        rc, base = container_run(container, args, logs / f"base_{mode}.log")
        if rc or base is None:
            raise RuntimeError(f"base build failed for {mode}: see {logs / f'base_{mode}.log'}")
        bases[mode] = base
        print(f"base {mode}: {base}", flush=True)
    domains = iter(range(cfg.get("domain0", 182), cfg.get("domain0", 182) + 1000))
    results, began = [], time.time()

    def one(job, domain):
        name = f"{job['mode']}_{job['profile']}"
        args = [recorder, "--base", bases[job["mode"]], "--profile", job["profile"], "--rate", str(job["rate"]),
                "--domain", str(domain), "--base-from-imu-source", manifest]
        args += ["--no-supervisor"] if job["mode"] == "bare" else []
        rc, out = container_run(container, args, logs / f"{name}.log")
        ok = rc == 0 and out is not None and (host(out) / "report.json").exists()
        return dict(job, slug=name, ok=ok, run=out and str(host(out)), error=None if ok else f"rc={rc}, see {name}.log")

    with ThreadPoolExecutor(cfg.get("parallel", 4)) as pool:
        futures = [pool.submit(one, job, next(domains)) for job in jobs]
        for future in as_completed(futures):
            results.append(future.result())
            r = results[-1]
            print(f"[{len(results)}/{len(jobs)}] {r['slug']}: {'ok' if r['ok'] else 'FAILED ' + r['error']}", flush=True)
    reports = []
    for r in results:
        if r["ok"]:
            report = json.loads((Path(r["run"]) / "report.json").read_text())
            reports.append(dict(report, profile=f"{r['mode']}:{report['profile']}", source=report["recorder"]))
    return results, reports, bases, time.time() - began


def rtk_markdown(suite, run, checks, criteria, reports, failures, wall_s, bases):
    lines = [f"# RTK / localization scorecard — {suite['suite']['name']}", "",
             f"Run `{run}` · {len(reports)} jobs · {wall_s:.0f} s wall · bases {bases}", ""]
    lines += ["| criterion | verdict |", "|---|---|"] + [f"| {k} | {'PASS' if v else 'FAIL'} |" for k, v in criteria.items()]
    lines += ["", "| criterion | check | profiles | jobs | value | threshold | pass | baseline |",
              "|---|---|---|---|---|---|---|---|"]
    for c in checks:
        base = "" if "baseline" not in c else f"{fmt(c['baseline'])}{' REGRESSED' if c['regressed'] else ''}"
        lines.append(f"| {c['criterion']} | {c['name']} | {', '.join(c['profiles'])} | {c['jobs']} | {fmt(c['value'])} "
                     f"| {c['reduce']} {c['op']} {c['threshold']} | {'yes' if c['passed'] else '**no**'} | {base} |")
    if failures:
        lines += ["", "Crashed jobs: " + ", ".join(f"`{f['slug']}` ({f['error']})" for f in failures)]
    lines += ["", "| profile | rate | h p95 / max cm | z p95 cm | edit window h max | after h max | held after s "
              "| permitted again s | faults | published h max while permitted cm | pose latency p95 / max s |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(reports, key=lambda r: r["profile"]):
        sup = r.get("supervision") or {}
        during, after = r.get("during") or {}, r.get("after") or {}
        faults = "; ".join(f"{f['t']:.0f} s {f['reason']}" for f in sup.get("faults", [])) or "-"
        lat = r.get("pose_latency_s") or {}
        lines.append(f"| {r['profile']} | {r['rate']} | {r['replay']['h_p95_cm']} / {r['replay']['h_max_cm']} "
                     f"| {r['replay']['z_p95_cm']} "
                     f"| {fmt(during.get('h_max_cm'))} | {fmt(after.get('h_max_cm'))} | {fmt(sup.get('motion_held_after_s'))} "
                     f"| {fmt(sup.get('motion_permitted_again_after_window_s'))} | {faults} "
                     f"| {fmt(sup.get('published_h_max_cm_while_permitted'))} | {fmt(lat.get('p95'))} / {fmt(lat.get('max'))} |")
    return "\n".join(lines) + "\n"


def metric(report, path):
    value = report
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return None  # e.g. no edit window on a nominal run
        value = value[key]
    return value


def score(suite, reports, baseline=None):
    """Evaluate every check over the reports whose profile (and source) match it."""
    base = {(c["criterion"], c["name"]): c for c in (baseline or {}).get("checks", [])}
    results = []
    for check in suite["checks"]:
        matched = [r for r in reports
                   if any(fnmatch.fnmatchcase(r["profile"], p) for p in check["profiles"])
                   and any(fnmatch.fnmatchcase(Path(r["source"]).name, s + "*") for s in check.get("sources", [""]))]
        values = [metric(r, check["metric"]) for r in matched]
        values = [v for v in values if v is not None]
        value = REDUCE[check["reduce"]](values) if values else None
        passed = value is not None and OPS[check["op"]](value, check["value"])
        row = dict(criterion=check["criterion"], name=check["name"], metric=check["metric"], op=check["op"],
                   threshold=check["value"], reduce=check["reduce"], profiles=check["profiles"], jobs=len(matched),
                   value=value, passed=passed)
        prior = base.get((check["criterion"], check["name"]))
        if prior is not None and prior.get("value") is not None and value is not None:
            better = 1 if check["op"] == ">=" else -1  # "==" checks are counts that should stay at 0
            delta = value - prior["value"]
            row.update(baseline=prior["value"], regressed=bool(better * delta < -check.get("tolerance", 0.0)))
        results.append(row)
    criteria = {}
    for row in results:
        criteria.setdefault(row["criterion"], []).append(row["passed"])
    return results, {k: all(v) for k, v in sorted(criteria.items())}


def fmt(v):
    if v is None:
        return "n/a"
    return f"{v:.4g}" if isinstance(v, float) else str(v)


def markdown(suite, run, checks, criteria, reports, failures, wall_s):
    from summarize_terrain_audit import HEADER, row

    lines = [f"# Terrain audit scorecard — {suite['suite']['name']}", "",
             f"Run `{run}` · {len(reports)} jobs · {wall_s:.0f} s wall · suite `{suite['path']}`", ""]
    lines += ["| criterion | verdict |", "|---|---|"]
    lines += [f"| {k} | {'PASS' if v else 'FAIL'} |" for k, v in criteria.items()]
    lines += ["", "| criterion | check | profiles | jobs | value | threshold | pass | baseline |",
              "|---|---|---|---|---|---|---|---|"]
    for c in checks:
        base = "" if "baseline" not in c else f"{fmt(c['baseline'])}{' REGRESSED' if c['regressed'] else ''}"
        lines.append(f"| {c['criterion']} | {c['name']} | {', '.join(c['profiles'])} | {c['jobs']} | {fmt(c['value'])} "
                     f"| {c['reduce']} {c['op']} {c['threshold']} | {'yes' if c['passed'] else '**no**'} | {base} |")
    if failures:
        lines += ["", "Crashed jobs: " + ", ".join(f"`{f['slug']}` ({f['error']})" for f in failures)]
    lines += ["", "| source | " + " | ".join(HEADER) + " |", "|" + "---|" * (len(HEADER) + 1)]
    for r in sorted(reports, key=lambda r: (Path(r["source"]).name, r["profile"])):
        lines.append(f"| {Path(r['source']).name[:17]} | " + " | ".join(row(r)) + " |")
    return "\n".join(lines) + "\n"


def memory_workers(per_job=6 * 1024**3):
    """Workers that fit in MemAvailable. A transit50 replay peaks near 4.6 GB; 19 of them
    exhausted the 62 GB host on Oct 21 (run 3afe2462) and took the session down with it."""
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return max(1, int(line.split()[1]) * 1024 // per_job)
    except OSError:
        pass
    return 4


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("suite", help="suite name under tools/audit_suites or a .toml path")
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--workers", type=int, default=0, help="0 = one per job, up to cores - 2")
    parser.add_argument("--reuse", type=Path, help="prior suite run whose cached layers may be re-scored")
    parser.add_argument("--baseline", type=Path, help="prior scorecard.json to flag regressions against")
    parser.add_argument("--no-save-layers", action="store_true", help="do not cache layers for later --reuse")
    args = parser.parse_args()
    archive = args.archive_root.resolve(strict=True)
    suite = load_suite(args.suite)
    baseline = json.loads(args.baseline.read_text()) if args.baseline else None
    if "rtk" in suite:
        run = allocate_run(archive, limit=1024**3)  # job outputs live in their own archive runs
        print(f"Run: {run}", flush=True)
        (run / "suite.json").write_text(json.dumps(suite, indent=2))
        results, reports, bases, wall_s = run_rtk(suite, archive, run)
        failures = [r for r in results if not r["ok"]]
        checks, criteria = score(suite, reports, baseline)
        card = dict(suite=suite["suite"]["name"], run=str(run), wall_s=round(wall_s, 1), criteria=criteria,
                    checks=checks, failures=failures, jobs=results, bases=bases)
        (run / "scorecard.json").write_text(json.dumps(card, indent=2))
        (run / "scorecard.md").write_text(rtk_markdown(suite, run, checks, criteria, reports, failures, wall_s, bases))
        check_run(run)
        print((run / "scorecard.md").read_text())
        sys.exit(1 if failures else 2 if not all(criteria.values()) or any(c.get("regressed") for c in checks) else 0)
    jobs = expand(suite, archive)
    run = allocate_run(archive, limit=16 * 1024**3)
    print(f"Run: {run}", flush=True)
    (run / "suite.json").write_text(json.dumps(dict(suite=suite, jobs=jobs, reuse=args.reuse and str(args.reuse)),
                                               indent=2))
    workers = args.workers or max(1, min(len(jobs), (os.cpu_count() or 4) - 2, memory_workers()))
    began = time.time()
    results = []
    (run / "jobs").mkdir()
    # spawn: workers must not inherit the parent's BLAS / native thread pools
    with ProcessPoolExecutor(workers, mp_context=multiprocessing.get_context("spawn")) as pool:
        futures = []
        for job in jobs:
            cache = args.reuse / "jobs" / job["slug"] / "layers.npz" if args.reuse else None
            futures.append(pool.submit(_work, job, str(archive), str(run / "jobs" / job["slug"]),
                                       cache and str(cache), not args.no_save_layers))
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            state = ("cached " if result.get("cached") else "") + (f"{result['wall_s']} s" if result["ok"]
                                                                   else "FAILED " + result["error"])
            print(f"[{len(results)}/{len(jobs)}] {result['slug']}: {state}", flush=True)
    wall_s = time.time() - began
    reports = [json.loads((run / "jobs" / r["slug"] / "report.json").read_text()) for r in results if r["ok"]]
    failures = [r for r in results if not r["ok"]]
    checks, criteria = score(suite, reports, baseline)
    card = dict(suite=suite["suite"]["name"], run=str(run), wall_s=round(wall_s, 1), criteria=criteria,
                checks=checks, failures=failures, jobs=results)
    (run / "scorecard.json").write_text(json.dumps(card, indent=2))
    (run / "scorecard.md").write_text(markdown(suite, run, checks, criteria, reports, failures, wall_s))
    check_run(run)
    print((run / "scorecard.md").read_text())
    if failures:
        sys.exit(1)
    if not all(criteria.values()) or any(c.get("regressed") for c in checks):
        sys.exit(2)


if __name__ == "__main__":
    main()
