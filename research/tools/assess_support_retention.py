#!/usr/bin/env python3
"""Falsify retention as the startup bottleneck. Optimistic diagnostic, not a policy.

Only past free cells are accumulated in the fixed local frame. No uncertainty
erosion or time expiry: deliberately an upper bound, NEVER motion authority.
Current obstacles/edge conflicts veto old support. Full raw/advisory replay
remains exact and every decision bin stays in the denominator.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from shapely import Polygon, difference, union_all

from archive_guard import allocate_run, check_run
from assess_startup_observation import exit_candidates
from local_terrain.contact_support import patch_polygons
from local_terrain.pose_lookup import PublicationPoseLookup
from local_terrain.support_sweep import swept_polygons
from replay_contact_advisory import evaluate
from replay_local_terrain import sha


class PastGroundUnion:
    def __init__(self):
        self.identity = self.reference = self.phase = None
        self.cells = set()
        self.stamp = -1

    def update(self, layers, record):
        identity = (record["session"], record["epoch"])
        reference = np.asarray(record["local_from_map"])
        origin, resolution = np.asarray(record["origin"]), record["resolution"]
        if record["publication_ns"] <= self.stamp or resolution != 0.1 or layers.shape != (60, 60, 6):
            raise ValueError("ordered fixed 60x60 / 10 cm diagnostic required")
        if self.identity is None:
            self.identity, self.reference, self.phase = identity, reference.copy(), origin.copy()
        if identity != self.identity or not np.array_equal(reference, self.reference):
            raise ValueError("support reference changed; no transporting old support")
        shift = np.rint((origin - self.phase) / resolution).astype(int)
        if not np.allclose(origin - self.phase, shift * resolution, atol=1e-12, rtol=0):
            raise ValueError("support grid phase changed")
        self.stamp = record["publication_ns"]
        # Retain at most this rolling grid. Leaving/reentering never resurrects.
        self.cells = {p for p in self.cells if all(shift <= p) and all(p < shift + 60)}
        free = np.isin(layers[:, :, 0], (0, 20))
        conflicts = (layers[:, :, 0] == 100) | ((layers[:, :, 0] == -1) & ~np.isin(layers[:, :, 1], (0, 4, 9, 12)))
        for y, x in np.argwhere(conflicts):
            self.cells.discard(tuple(shift + [x, y]))
        self.cells.update(tuple(shift + [x, y]) for y, x in np.argwhere(free))
        result = layers.copy()
        added = 0
        for p in self.cells:
            x, y = np.asarray(p) - shift
            if result[y, x, 0] == -1 and not conflicts[y, x]:
                result[y, x, 0] = 20  # Diagnostic copy only; never published.
                added += 1
        return result, added


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    memory, rows = PastGroundUnion(), []
    with (output / "decisions.jsonl").open("x") as sink:

        def observe(runtime, layers, record, actual):
            accumulated, added = memory.update(layers, record)
            row = dict(query_ns=actual["query_ns"], added_support_cells=added, motions=None)
            if actual["motions"] is not None:
                advisory = runtime.advisory
                lookup = PublicationPoseLookup(runtime.timeline, actual["query_ns"])
                startup_pose = (
                    np.asarray(record["local_from_map"])
                    @ lookup.associate(advisory.memory.anchors[0].stamp_ns).map_from_lidar
                    @ np.linalg.inv(runtime.timeline.extrinsic)
                    @ np.linalg.inv(runtime.node.core.base_from_imu)
                )
                xyz = np.c_[advisory.memory.footprint, np.full(len(advisory.memory.footprint), advisory.memory.floor_z)]
                startup = Polygon((xyz @ startup_pose[:3, :3].T + startup_pose[:3, 3])[:, :2])
                assumed = union_all(
                    [startup, *[Polygon(p) for p in patch_polygons(advisory.context.local_from_base, advisory.patches)]]
                )
                decisions, original = {}, advisory.layers
                try:
                    for name, relative in exit_candidates().items():
                        swept = union_all(
                            [Polygon(p) for p in swept_polygons(advisory.context.poses(relative), advisory.patches)]
                        )
                        outside = float(difference(swept, assumed).area)
                        advisory.layers = original
                        baseline = advisory.query(relative, query_ns=actual["query_ns"])
                        advisory.layers = accumulated
                        optimistic = advisory.query(relative, query_ns=actual["query_ns"])
                        decisions[name] = dict(
                            baseline=baseline["state"],
                            upper_bound=optimistic["state"],
                            baseline_exit=baseline["state"] == "available" and outside > 0.001,
                            upper_bound_exit=optimistic["state"] == "available" and outside > 0.001,
                            beyond_assumed_m2=outside,
                            unresolved_m2=optimistic.get("new_ground_area_m2"),
                        )
                finally:
                    advisory.layers = original
                row["motions"] = decisions
            sink.write(json.dumps(row) + "\n")
            rows.append(row)

        exact = evaluate(args.source, args.archive_root, output, observation_callback=observe)
    batches = [r for r in rows if r["motions"] is not None]
    report = dict(
        completed=True,
        acceptance=False,
        navigation_authority=False,
        diagnostic="causal past-ground upper bound: no age or uncertainty allowance; not a support policy",
        exact_replay=exact,
        decisions=len(batches),
        baseline_exit_decisions=sum(any(d["baseline_exit"] for d in r["motions"].values()) for r in batches),
        upper_bound_exit_decisions=sum(any(d["upper_bound_exit"] for d in r["motions"].values()) for r in batches),
        final_added_support_cells=rows[-1]["added_support_cells"],
        output_sha256={"decisions.jsonl": sha(output / "decisions.jsonl")},
        source_sha256=sha(Path(__file__)),
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    check_run(output)
    print(json.dumps({k: v for k, v in report.items() if k != "exact_replay"}, indent=2))


if __name__ == "__main__":
    main()
