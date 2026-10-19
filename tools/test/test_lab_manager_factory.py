"""No Isaac required: unselected worlds must not be imported as a side effect."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


def factory_module():
    path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/lab_manager_factory.py"
    spec = importlib.util.spec_from_file_location("lab_factory_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("world", ["Lunalab", "Lunaryard", "LargeScale"])
def test_only_requested_environment_is_loaded_with_original_arguments(monkeypatch, world):
    module = factory_module()
    imports, constructions = [], []
    expected_module, expected_class = module.ROS2_LMF._lab_managers[world]

    def load(name):
        assert name == expected_module
        imports.append(name)
        return SimpleNamespace(**{expected_class: lambda **kwargs: constructions.append(kwargs) or kwargs})

    monkeypatch.setattr(module, "import_module", load)
    cfg = dict(environment=dict(name=world, seed=43))
    result = module.ROS2_LMF(cfg, sentinel="kept")
    assert result == dict(environment_cfg=cfg["environment"], sentinel="kept")
    assert imports == [expected_module] and constructions == [result]
    assert result["environment_cfg"] is cfg["environment"]


def test_explicit_registration_and_unknown_environment(monkeypatch):
    module = factory_module()
    monkeypatch.setattr(module, "import_module", lambda _: pytest.fail("unexpected import"))
    module.ROS2_LMF.register("Custom", lambda **kwargs: kwargs)
    assert module.ROS2_LMF(dict(environment=dict(name="Custom")))["environment_cfg"] == dict(name="Custom")
    with pytest.raises(KeyError):
        module.ROS2_LMF(dict(environment=dict(name="missing")))


def test_manager_no_longer_eagerly_imports_all_worlds():
    import ast
    path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/simulation_manager_ros2.py"
    imports = [node.module for node in ast.walk(ast.parse(path.read_text())) if isinstance(node, ast.ImportFrom)]
    assert "src.environments_wrappers.ros2.lab_manager_factory" in imports
    assert not any(name and name.endswith((".lunalab_ros2", ".lunaryard_ros2", ".largescale_ros2")) for name in imports)
