"""Immutable candidate-library provenance, independent of a host compiler."""

import json
from pathlib import Path
import sys
import subprocess

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from build_local_terrain import digest, validate_build


def test_diagnostic_deferred_abi_rejected_before_allocating_archive():
    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).parents[1] / "build_local_terrain.py"),
            "--defer-native",
            "--patch-normal-cache",
            "--diagnostics",
        ],
        text=True,
        capture_output=True,
    )
    assert result.returncode == 2
    assert "omit --defer-native" in result.stderr
    assert "Run:" not in result.stdout


def build(tmp_path):
    archive = tmp_path / "archive"
    run = archive / "candidate"
    run.mkdir(parents=True)
    source = run / "source.cpp"
    source.write_text("// fixture\n")
    library = run / "liblocal_terrain.so"
    library.write_bytes(b"fixture library, not executable")
    (run / "build.json").write_text(
        json.dumps(
            dict(
                schema=1,
                library=library.name,
                library_sha256=digest(library),
                source_sha256={source.name: digest(source)},
            )
        )
    )
    return archive, run, library


def test_candidate_source_and_library_hashes_retained(tmp_path):
    archive, run, library = build(tmp_path)
    result = validate_build(library, archive)
    assert set(result) == {str(library), str(run / "build.json"), str(run / "source.cpp")}


@pytest.mark.parametrize("fault", ["library", "source", "escape", "wrong_name", "schema"])
def test_candidate_mutation_and_escape_rejected(tmp_path, fault):
    archive, run, library = build(tmp_path)
    if fault == "library":
        library.write_bytes(b"changed")
    elif fault == "source":
        (run / "source.cpp").write_text("changed")
    else:
        manifest = json.loads((run / "build.json").read_text())
        if fault == "escape":
            outside = archive / "external.cpp"
            outside.write_text("outside candidate build")
            manifest["source_sha256"] = {"../external.cpp": digest(outside)}
        elif fault == "schema":
            manifest["schema"] = 2
        else:
            manifest["library"] = "other.so"
        (run / "build.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        validate_build(library, archive)


def test_library_outside_declared_archive_rejected(tmp_path):
    archive, run, library = build(tmp_path)
    other = tmp_path / "other"
    other.mkdir()
    with pytest.raises(ValueError):
        validate_build(library, other)
