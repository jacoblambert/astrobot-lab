import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from terrain_entrypoint_archive import historical_entrypoint, source_hash


def test_historical_ros_source_is_content_addressed_and_cannot_replace_mapper(tmp_path):
    source = tmp_path / "revision-abc" / "live_local_terrain.py"
    source.parent.mkdir()
    source.write_text("# frozen live entrypoint\n")
    digest = source_hash(source)
    index = tmp_path / "terrain_entrypoints"
    index.mkdir()
    (index / (digest + ".py")).symlink_to(source)
    assert historical_entrypoint(Path("live_local_terrain.py"), digest, tmp_path) == source
    with pytest.raises(ValueError, match="only"):
        historical_entrypoint(Path("core.py"), digest, tmp_path)
    source.write_text("# modified\n")
    with pytest.raises(ValueError, match="changed"):
        historical_entrypoint(Path("live_local_terrain.py"), digest, tmp_path)
