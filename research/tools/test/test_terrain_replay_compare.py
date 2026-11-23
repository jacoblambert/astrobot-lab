import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from compare_terrain_replays import compare_replays


@pytest.mark.parametrize("changed", [None, "provenance", "frame", "array", "failure", "grid", "pose_mode", "source"])
def test_exact_replay_guard(tmp_path, changed):
    runs = [tmp_path / "baseline", tmp_path / "candidate"]
    for i, run in enumerate(runs):
        run.mkdir()
        manifest = dict(input_sha256={"scan": "frozen"}, reference_sha256={"tile": "frozen"},
                        variants=["test"], multiview=True, visibility_denominator="fixed")
        summary = dict(frames=[dict(stamp=1., cost_reason_sha256="frozen")], latency_p95_ms=10. + i)
        layers = np.array([0., np.nan])
        if i == 1:
            if changed == "provenance":
                manifest["reference_sha256"]["tile"] = "changed"
            elif changed == "frame":
                summary["frames"][0]["stamp"] = 2.
            elif changed == "array":
                layers[0] = 100.
            elif changed == "failure":
                (run / "failure.json").write_text("{}")
            elif changed == "grid":
                manifest["terrain_geometry"]=dict(origin=[-9.975,-9.975],resolution=.1,width=200,height=200)
            elif changed == "pose_mode":
                manifest["pose_sensitivity"] = {"mode": "first_published"}
            elif changed == "source":
                manifest["source"] = "native_tick_offline_pose_sensitivity"
        (run / "manifest.json").write_text(json.dumps(manifest))
        (run / "summary.json").write_text(json.dumps(summary))
        np.savez_compressed(run / "test.npz", layers=layers)
    if changed:
        with pytest.raises(ValueError):
            compare_replays(*runs)
    else:
        assert compare_replays(*runs)["exact_match"]
