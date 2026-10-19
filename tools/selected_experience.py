"""Opt-in lean Isaac startup: preserve upstream settings, select dependencies.

Generated experiences and their inputs live only in a fresh external archive.
This is an experiment, not a change to the default simulator application.
"""

import copy
import hashlib
import json
from pathlib import Path

import toml

REQUIRED = (
    "isaacsim.core.api", "isaacsim.core.nodes", "isaacsim.core.simulation_manager",
    "isaacsim.core.utils", "isaacsim.core.version", "isaacsim.core.deprecation_manager",
    "isaacsim.robot.wheeled_robots", "isaacsim.robot.schema", "isaacsim.sensors.camera",
    "isaacsim.sensors.physics", "isaacsim.sensors.physx", "isaacsim.simulation_app",
    "isaacsim.storage.native", "omni.isaac.dynamic_control", "omni.kit.loop-isaac",
    "omni.graph.action", "omni.graph.nodes", "omni.hydra.rtx", "omni.hydra.usdrt_delegate",
    "omni.kit.material.library", "omni.kit.viewport.window", "omni.physics.physx",
    "omni.physics.stageupdate", "omni.physx.tensors", "omni.replicator.core",
    "omni.syntheticdata", "semantics.schema.editor", "omni.warp.core",
)


def merge(base, overlay):
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def absolute_app_tokens(value, original_app_dir):
    if isinstance(value, str):
        return value.replace("${app}", str(original_app_dir))
    if isinstance(value, list):
        return [absolute_app_tokens(item, original_app_dir) for item in value]
    if isinstance(value, dict):
        return {key: absolute_app_tokens(item, original_app_dir) for key, item in value.items()}
    return value


def build_config(base, python_app, original_app_dir):
    if set(python_app["dependencies"]) != {"isaacsim.exp.base"}:
        raise ValueError("unexpected upstream Python experience dependencies")
    if any(name not in base["dependencies"] for name in REQUIRED):
        raise ValueError("upstream experience missing required selected dependency")
    settings = absolute_app_tokens(merge(base["settings"], python_app["settings"]), original_app_dir)
    return dict(
        package=dict(title="Selected headless diagnostic", version="0.1.0"),
        dependencies={name: copy.deepcopy(base["dependencies"][name]) for name in REQUIRED},
        settings=settings,
    )


def prepare(app_dir=Path("/isaac-sim/apps"), archive_root=Path("/archive")):
    from archive_guard import allocate_run, check_run

    sources = [app_dir / "isaacsim.exp.base.kit", app_dir / "isaacsim.exp.base.python.kit"]
    payloads = [path.read_text() for path in sources]
    config = build_config(*(toml.loads(value) for value in payloads), app_dir)
    output = allocate_run(archive_root, limit=1024**3)
    for path, payload in zip(sources, payloads):
        with (output / path.name).open("x") as stream:
            stream.write(payload)
    experience = output / "selected_headless.kit"
    with experience.open("x") as stream:
        stream.write(toml.dumps(config))
    if toml.loads(experience.read_text()) != config:
        raise ValueError("generated experience did not round-trip exactly")
    metadata = dict(
        profile="lean_v1", acceptance=False,
        removed_direct_dependencies=sorted(set(toml.loads(payloads[0])["dependencies"]) - set(REQUIRED)),
        settings_preserved=True, source_sha256={str(path): hashlib.sha256(payload.encode()).hexdigest()
                                               for path, payload in zip(sources, payloads)},
        builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        experience_sha256=hashlib.sha256(experience.read_bytes()).hexdigest(),
    )
    with (output / "experience.json").open("x") as stream:
        json.dump(metadata, stream, indent=2)
    check_run(output)
    print(f"SELECTED_EXPERIENCE {experience}", flush=True)
    return experience
