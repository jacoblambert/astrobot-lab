"""Host-only provenance contracts; no ROS, GPU or archive required."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from write_run_manifest import mapped_library_paths, loaded_libraries


def test_mapped_libraries_preserve_spaces_deduplicate_and_reject_deleted():
    row = "7f-8f r-xp 0000 08:01 123 "
    assert mapped_library_paths(row + "/a b/libtest.so.1\n" + row + "/a b/libtest.so.1\n"
                                + row + "[heap]\n") == ["/a b/libtest.so.1"]
    with pytest.raises(ValueError, match="deleted"):
        mapped_library_paths(row + "/plugin.so (deleted)")


def test_loaded_code_hashes_actual_bytes(tmp_path):
    library = tmp_path / "plugin.so"
    library.write_bytes(b"fixture")
    (tmp_path / "maps").write_text(f"7f-8f r-xp 0000 08:01 123 {library}\n")
    result = loaded_libraries(tmp_path)
    assert len(result["libraries"]) == 1
    assert result["libraries"][0]["size"] == 7
    assert len(result["libraries"][0]["sha256"]) == 64


def test_loaded_code_set_change_is_not_attested(tmp_path, monkeypatch):
    (tmp_path / "maps").write_text("")
    calls = []
    def read(path, *args, **kwargs):
        calls.append(path)
        return "" if len(calls) == 1 else "7f-8f r-xp 0000 08:01 123 /new.so\n"
    monkeypatch.setattr(Path, "read_text", read)
    with pytest.raises(ValueError, match="changed"):
        loaded_libraries(tmp_path)
