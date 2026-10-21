#!/usr/bin/env python3
"""Tabulate audit_terrain_robustness.py report.json files (one row per run)."""

import argparse
import json
from pathlib import Path


def nav2(pol, name):
    if name not in pol:
        return "-"
    p = pol[name]
    return (f'{100 * p["frames_with_action"]:.0f}/{p["chosen_unsafe_corridors"]}/{p["chosen_unsafe_large"]}'
            f'/{100 * p["hidden_hazard_rate"]:.2f}')


def row(report):
    c2, c1, c4 = report["c2_flat"], report["c1_gaps"], report["c4_recall"]
    big = c4[">=0.20m"]
    pol = c1["policies"]
    return [
        report["profile"],
        f'{c2["observed_fraction"]:.2f}',
        f'{100 * c2["phantom_of_observed"]:.2f}',
        f'{100 * c2["open_floor_phantom_of_observed"]:.2f}',
        f'{100 * c2["frames_with_phantom_ahead"]:.1f}',
        f'{100 * c2["frames_with_open_floor_phantom_near"]:.1f}',
        f'{100 * c1["near_unknown_fraction"]:.0f}',
        f'{100 * pol["strict"]["frames_with_action"]:.0f}',
        f'{100 * pol["bounded"]["frames_with_action"]:.0f}/{pol["bounded"]["chosen_unsafe_corridors"]}/{pol["bounded"]["chosen_possible_unsafe_corridors"]}/{pol["bounded"]["available_corridors"]}',
        nav2(pol, "nav2_isolated"),
        nav2(pol, "nav2_bounded_shadow"),
        f'{pol["optimistic"]["chosen_unsafe_corridors"]}/{pol["optimistic"]["chosen_possible_unsafe_corridors"]}/{pol["optimistic"]["available_corridors"]}',
        f'{big["ever_marked"]}/{big["objects"]}',
        f'{c4[">=0.10m"]["ever_marked"]}/{c4[">=0.10m"]["objects"]} {c4[">=0.10m"]["within_2m_unmarked_frames"]}/{c4[">=0.10m"]["within_2m_frames"]}',
        f'{big["within_2m_unmarked_frames"]}/{big["within_2m_frames"]}',
        f'{big["danger_unmarked_frames"]}',
        str(c1["near_cells_of_unmarked_large_objects"]),
    ]


HEADER = ["profile", "flat obs", "phantom %obs", "open-floor phantom %obs", "phantom-ahead %fr",
          "open-floor phantom-near %fr", "near unk %", "strict act %fr", "bounded act%fr/unsafe/possible/avail",
          "nav2 isolated act%fr/unsafe/unsafe>=20cm/hidden%",
          "nav2 bounded+shadow act%fr/unsafe/unsafe>=20cm/hidden%", "optim unsafe/possible/avail", ">=20cm marked",
          ">=10cm marked, unmarked<2m fr",
          ">=20cm unmarked<2m fr", "danger unmarked fr", "hidden large cells"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path, help="audit run directories or report.json files")
    args = parser.parse_args()
    rows = []
    for p in args.runs:
        p = p / "report.json" if p.is_dir() else p
        if p.exists():
            r = json.loads(p.read_text())
            rows.append((Path(r["source"]).name[:17], row(r)))
    print("| source | " + " | ".join(HEADER) + " |")
    print("|" + "---|" * (len(HEADER) + 1))
    for src, r in sorted(rows, key=lambda x: (x[0], x[1][0])):
        print(f"| {src} | " + " | ".join(r) + " |")


if __name__ == "__main__":
    main()
