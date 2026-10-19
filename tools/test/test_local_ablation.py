"""One-factor comparisons must reject changed exposure, inputs or provenance."""

import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from compare_local_ablation import compare
from replay_local_terrain import sha
from test_local_build import build


def fixture_pair(tmp_path):
    paths = [tmp_path / "left", tmp_path / "right"]
    for run, policy in zip(paths, ("coherent", "frozen_observation")):
        run.mkdir()
        (run / "source").mkdir()
        (run / "source/core.py").write_text("# frozen test source\n")
        (run / "manifest.json").write_text(
            json.dumps({"source_sha256": {"/workspace/tools/core.py": sha(run / "source/core.py")}})
        )
        row = dict(
            evidence_ns=0,
            publication_ns=200_000_000,
            session="s",
            epoch=0,
            revision=1,
            local_from_base=[[1]],
            local_from_map=[[1]],
            origin=[-3, -3],
            associations=[dict(sequence=0, acquisition_ns=0)],
            layers_sha256="a",
            availability={"fixed": dict(queries=10, free=7, occupied=1, unknown=2, outside=1)},
        )
        (run / "projections.jsonl").write_text(json.dumps(row) + "\n")
        report = dict(
            diagnostic_completed=True,
            policy="local",
            grid_motion="rolling",
            instrumented=False,
            input_sha256={"capture": "hash"},
            library_sha256="lib",
            source_failed=True,
            terminal_fault="retained source failure",
            bounded_prefix=True,
            requested_max_projections=1,
            projections=1,
            window_s=2,
            grid_cells=60,
            observation_policy=policy,
            compute_ms={"p95": 100},
            peak_rss_mib=150,
            output_sha256=sha(run / "projections.jsonl"),
        )
        (run / "report.json").write_text(json.dumps(report))
    return paths


def test_matched_ablation_keeps_failures_and_denominators(tmp_path):
    result = compare(*fixture_pair(tmp_path))
    assert result["factor"] == "observation_policy"
    assert result["projections"] == result["identical_layers"] == 1
    assert result["source_failed"] and result["bounded_prefix"]
    assert not result["acceptance"] and not result["terrain_truth_evaluated"]
    assert result["availability"][0]["fixed"]["outside"] == 1
    assert result["availability"][0]["fixed"]["free_fraction"] == 0.7


def test_rolling_incremental_is_not_mislabeled_an_exact_rebuild(tmp_path):
    paths = fixture_pair(tmp_path)
    report_path = paths[1] / "report.json"
    report = json.loads(report_path.read_text())
    report.update(observation_policy="coherent", map_policy="rolling_incremental")
    report_path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="same local reconstruction path"):
        compare(*paths, exact=True)


@pytest.mark.parametrize("changed", [False, True])
def test_backend_comparison_requires_exact_layers(tmp_path, changed):
    paths = fixture_pair(tmp_path)
    for path, backend in zip(paths, ("reference", "prepared")):
        report_path = path / "report.json"
        report = json.loads(report_path.read_text())
        report.update(backend=backend, observation_policy="coherent")
        if changed and backend == "prepared":
            projection_path = path / "projections.jsonl"
            row = json.loads(projection_path.read_text())
            row["layers_sha256"] = "changed"
            projection_path.write_text(json.dumps(row) + "\n")
            report["output_sha256"] = sha(projection_path)
        report_path.write_text(json.dumps(report))
    if changed:
        with pytest.raises(ValueError, match="exact backend equivalence"):
            compare(*paths)
    else:
        assert compare(*paths)["factor"] == "backend"


def test_exact_implementation_change_allows_new_sources_not_changed_policy(tmp_path):
    paths = fixture_pair(tmp_path)
    report_path = paths[1] / "report.json"
    report = json.loads(report_path.read_text())
    # Algorithm policies must still be identical, even under --exact.
    with pytest.raises(ValueError, match="algorithm factors"):
        compare(*paths, exact=True)
    report["observation_policy"] = "coherent"
    report_path.write_text(json.dumps(report))
    retained = paths[1] / "source/core.py"
    retained.write_text("# equivalent implementation\n")
    (paths[1] / "manifest.json").write_text(json.dumps({"source_sha256": {"/workspace/tools/core.py": sha(retained)}}))
    assert compare(*paths, exact=True)["identical_layers"] == 1
    row_path = paths[1] / "projections.jsonl"
    row = json.loads(row_path.read_text())
    row["layers_sha256"] = "regression"
    row_path.write_text(json.dumps(row) + "\n")
    report["output_sha256"] = sha(row_path)
    report_path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="exact backend equivalence"):
        compare(*paths, exact=True)


@pytest.mark.parametrize("fault", [None, "missing", "incomplete", "unrelated", "mutated"])
def test_changed_library_requires_complete_matching_build(tmp_path, fault):
    from build_local_terrain import validate_build

    paths = fixture_pair(tmp_path)
    archive, run, library = build(tmp_path)
    report_path = paths[1] / "report.json"
    report = json.loads(report_path.read_text())
    report.update(observation_policy="coherent", library_sha256=sha(library))
    report["candidate_build_sha256"] = validate_build(library, archive)
    if fault == "missing":
        report.pop("candidate_build_sha256")
    elif fault == "incomplete":
        report["candidate_build_sha256"].pop(str(run / "source.cpp"))
    elif fault == "unrelated":
        report["library_sha256"] = "unrelated"
    elif fault == "mutated":
        library.write_bytes(b"changed")
    report_path.write_text(json.dumps(report))
    if fault:
        with pytest.raises(ValueError):
            compare(*paths, exact=True)
    else:
        assert compare(*paths, exact=True)["identical_layers"] == 1
        with pytest.raises(ValueError):
            compare(*paths)


@pytest.mark.parametrize(
    "key,value",
    [
        ("window_s", 5),
        ("input_sha256", {}),
        ("projections", 2),
        ("source_failed", False),
        ("bounded_prefix", False),
        ("library_sha256", "other"),
        ("instrumented", True),
        ("policy", "first_published"),
        ("observation_policy", "coherent"),
    ],
)
def test_unmatched_report_rejected(tmp_path, key, value):
    paths = fixture_pair(tmp_path)
    path = paths[1] / "report.json"
    report = json.loads(path.read_text())
    report[key] = value
    path.write_text(json.dumps(report))
    with pytest.raises(ValueError):
        compare(*paths)


@pytest.mark.parametrize("change", ["publication", "rover", "dose", "denominator", "hash", "tail", "source"])
def test_projection_or_retained_source_tampering_rejected(tmp_path, change):
    paths = fixture_pair(tmp_path)
    path = paths[1] / "projections.jsonl"
    row = json.loads(path.read_text())
    if change == "publication":
        row["publication_ns"] += 1
    elif change == "rover":
        row["local_from_base"] = [[2]]
    elif change == "dose":
        row["associations"] = []
    elif change == "denominator":
        row["availability"]["fixed"]["queries"] -= 1
    elif change == "source":
        (paths[1] / "source/core.py").write_text("changed")
    path.write_text("" if change == "tail" else json.dumps(row) + "\n ")
    if change != "hash":
        report_path = paths[1] / "report.json"
        report = json.loads(report_path.read_text())
        report["output_sha256"] = sha(path)
        report_path.write_text(json.dumps(report))
    with pytest.raises(ValueError):
        compare(*paths)
