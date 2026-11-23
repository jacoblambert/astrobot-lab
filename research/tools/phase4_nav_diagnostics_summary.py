#!/usr/bin/env python3
"""Summarize Phase 4 navigation diagnostics across gate outputs/artifact dirs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_error": str(exc), "_path": str(path)}


def load_goal_events(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    events = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def merge_counts(records: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for record in records:
        classes = record.get("failure_class_counts") or record.get("failure_classes") or {}
        for key, value in classes.items():
            counts[key] = counts.get(key, 0) + int(value)
    return counts


def attempt_row(parent: dict[str, Any], attempt: dict[str, Any]) -> dict[str, Any]:
    verdict = attempt.get("verdict") or {}
    metrics = verdict.get("metrics") or {}
    status = attempt.get("latest_status") or {}
    artifact_dir = Path(attempt.get("artifact_dir", ""))
    goal_events = load_goal_events(artifact_dir / "goal_events.jsonl")
    efficiencies = [
        float(item["path_efficiency_ratio"])
        for item in goal_events
        if item.get("path_efficiency_ratio") is not None
    ]
    return {
        "mode": parent.get("mode"),
        "seed": parent.get("seed"),
        "attempt": attempt.get("attempt"),
        "result": "PASS" if verdict.get("pass") else "FAIL",
        "reasons": ",".join(verdict.get("reasons") or []),
        "return_reason": status.get("return_reason"),
        "coverage": metrics.get("coverage_gain_fraction", status.get("mission_coverage_fraction")),
        "home_dist": metrics.get("home_dist"),
        "goals": status.get("goals"),
        "successes": status.get("successes"),
        "failures": status.get("failures"),
        "precheck_rejections": status.get("precheck_rejections"),
        "snapshots": attempt.get("snapshot_count"),
        "failure_classes": attempt.get("failure_class_counts") or {},
        "path_eff_median": sorted(efficiencies)[len(efficiencies) // 2] if efficiencies else None,
        "artifact_dir": str(artifact_dir) if artifact_dir else "",
    }


def format_value(value: Any, width: int) -> str:
    if isinstance(value, float):
        text = f"{value:.3f}"
    elif isinstance(value, dict):
        text = ",".join(f"{k}:{v}" for k, v in sorted(value.items()))
    elif value is None:
        text = ""
    else:
        text = str(value)
    return text[:width].ljust(width)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="*", default=["rosbags/phase4_gate_suite.json"])
    args = parser.parse_args()

    rows: list[dict[str, Any]] = []
    for raw in args.paths:
        path = Path(raw)
        data = load_json(path)
        if "results" in data:
            for result in data["results"]:
                for attempt in result.get("attempt_records", []):
                    rows.append(attempt_row(result, attempt))
        elif "attempt_records" in data:
            for attempt in data["attempt_records"]:
                rows.append(attempt_row(data, attempt))
        elif "verdict" in data:
            pseudo_parent = {"mode": path.stem, "seed": ""}
            rows.append(
                attempt_row(
                    pseudo_parent,
                    {
                        "attempt": 1,
                        "verdict": data.get("verdict"),
                        "latest_status": data.get("latest_status"),
                        "failure_class_counts": data.get("failure_class_counts"),
                        "snapshot_count": data.get("snapshot_count"),
                    },
                )
            )

    columns = [
        ("mode", 15),
        ("seed", 5),
        ("attempt", 7),
        ("result", 6),
        ("coverage", 8),
        ("home_dist", 9),
        ("goals", 6),
        ("successes", 5),
        ("failures", 5),
        ("precheck_rejections", 8),
        ("snapshots", 5),
        ("failure_classes", 42),
        ("reasons", 32),
    ]
    print(" ".join(name[:width].ljust(width) for name, width in columns))
    for row in rows:
        print(" ".join(format_value(row.get(name), width) for name, width in columns))
    print()
    print("aggregate_failure_classes:", json.dumps(merge_counts(rows), sort_keys=True))


if __name__ == "__main__":
    main()
