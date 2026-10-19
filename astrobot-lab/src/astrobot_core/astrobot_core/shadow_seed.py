"""Explicit experiment-only initial state, never a navigation GT correction."""
import math
import re


def shadow_dump_path(seed_path, epoch):
    import json
    from pathlib import Path
    run = Path(seed_path).resolve(strict=True).parent
    contract = json.loads((run / "archive_contract.json").read_text())
    root = Path(contract["root"]).resolve(strict=True)
    if (contract.get("external_required") is not True or contract.get("run") != run.name
            or run.parent != root or root.stat().st_dev != contract["device"]
            or root.stat().st_dev == Path("/").stat().st_dev or type(epoch) is not int or epoch < 1):
        raise ValueError("shadow dump requires the recorder's current external archive lease")
    return run / f"glim_dump_epoch_{epoch}"


def validate_shadow_seed(value, shadow_mode, now):
    if not shadow_mode:
        raise ValueError("GT seed is forbidden outside shadow acquisition")
    pose = value.get("pose", [])
    if (value.get("schema") != 1 or value.get("one_time_only") is not True
            or value.get("stationary_hold_required") is not True
            or not re.fullmatch(r"revision-[0-9a-f]{32}", value.get("capture", ""))
            or len(pose) != 7 or not all(math.isfinite(x) for x in pose)
            or abs(sum(x*x for x in pose[3:])-1.) > 1e-5
            or not math.isfinite(value.get("stamp", math.nan)) or not 0 <= now-value["stamp"] <= 30.):
        raise ValueError("invalid/stale experiment-only seed")
    return list(pose)
