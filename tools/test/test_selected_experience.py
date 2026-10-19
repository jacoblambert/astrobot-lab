from pathlib import Path
import sys

import pytest
import toml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from selected_experience import REQUIRED, build_config


def inputs():
    base = dict(dependencies={name: {} for name in (*REQUIRED, "unused.gui")},
                settings=dict(app=dict(settings=dict(persistent=True), folder="${app}/../exts"),
                              physics=dict(dt=1/60, gravity=1.62), other=["${app}", "unchanged"]))
    python = dict(dependencies={"isaacsim.exp.base": {}},
                  settings=dict(app=dict(settings=dict(persistent=False))))
    return base, python


def test_preserves_nested_upstream_settings_without_mutating_inputs():
    base, python = inputs()
    result = build_config(base, python, Path("/isaac-sim/apps"))
    assert result["settings"]["physics"] == base["settings"]["physics"]
    assert result["settings"]["app"] == dict(settings=dict(persistent=False), folder="/isaac-sim/apps/../exts")
    assert result["settings"]["other"] == ["/isaac-sim/apps", "unchanged"]
    assert set(result["dependencies"]) == set(REQUIRED)
    assert base["settings"]["app"]["settings"]["persistent"] is True
    assert toml.loads(toml.dumps(result)) == result


def test_changed_upstream_dependency_contract_is_rejected():
    base, python = inputs()
    python["dependencies"]["unexpected"] = {}
    with pytest.raises(ValueError, match="unexpected upstream"):
        build_config(base, python, Path("/isaac-sim/apps"))
    base, python = inputs()
    del base["dependencies"][REQUIRED[0]]
    with pytest.raises(ValueError, match="missing required"):
        build_config(base, python, Path("/isaac-sim/apps"))
