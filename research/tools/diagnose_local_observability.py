#!/usr/bin/env python3
"""Archived optimistic flat-ground retention bound using actual mount/beam angles.

Counterfactual level flat ground, constant attitude/forward speed, continuous
angular cone and no occlusion. This is not a new capture, terrain label, physical
contact-patch model or moving sensing qualification. Failed-source status stays.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.observability import minimum_forward_history


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("loaded_contact", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--diagnose-failed-prefix", action="store_true")
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        archive = args.archive_root.resolve(strict=True)
        capture, loaded = [p.resolve(strict=True) for p in (args.capture, args.loaded_contact)]
        if not all(p.is_relative_to(archive) for p in (capture, loaded)):
            raise ValueError("diagnostic inputs must reside on the declared archive")
        failed = (capture / "failure.json").exists()
        if failed and not args.diagnose_failed_prefix:
            raise ValueError("failed capture requires explicit diagnostic mode")
        paths = [
            capture / "manifest.json",
            capture / "native_timing_audit.json",
            loaded / "manifest.json",
            loaded / "report.json",
            loaded / "contacts.jsonl",
        ]
        hashes = {str(p): sha(p) for p in paths}
        manifest = json.loads(paths[0].read_text())
        frame = json.loads(paths[1].read_text())["frames"][0]
        native = (capture / "native_chunks" / frame["file"]).resolve(strict=True)
        if not native.is_relative_to(capture / "native_chunks") or sha(native) != frame["sha256"]:
            raise ValueError("native angle source changed or escaped capture")
        hashes[str(native)] = sha(native)
        contract, report = [json.loads(p.read_text()) for p in paths[2:4]]
        if (loaded / "failure.json").exists() or not report["fixture_passed"]:
            raise ValueError("successful loaded characterization required")
        if sha(paths[4]) != report["contacts_sha256"]:
            raise ValueError("loaded contact evidence changed")
        robot_hashes = [
            digest
            for path, digest in manifest["source_asset_sha256"].items()
            if Path(path).name == "ros2_husky_PhysX_vlp16_mono_depth_imu.usd"
        ]
        if robot_hashes != [contract["asset_sha256"]]:
            raise ValueError("capture and loaded fixture use different rover assets")
        rows = [json.loads(line) for line in paths[4].read_text().splitlines()]
        if len(rows) != 480 or [row["step"] for row in rows] != list(range(1, 481)):
            raise ValueError("incomplete loaded characterization")
        last = rows[-1]
        positions = np.asarray(last["positions"])
        q = last["orientations_wxyz"][0]
        rotation = Rotation.from_quat([*q[1:], q[0]])
        wheel_xy = rotation.inv().apply(positions[1:] - positions[0])[:, :2]
        with np.load(native, allow_pickle=False) as data:
            zenith = data["zenith"].astype(float)
        sensor = manifest["sensor"]
        settings = sensor["sensor_attributes"]
        speed = 0.1  # existing declared forward experimental envelope
        results = []
        for name, xy in zip(contract["wheels"], wheel_xy):
            distance = minimum_forward_history(
                sensor["base_from_lidar"],
                report["mean_base_height_m"],
                xy,
                zenith,
                minimum=float(settings["minRange"]),
                maximum=float(settings["maxRange"]),
            )
            seconds = None if distance is None else distance / speed
            results.append(
                dict(
                    wheel=name,
                    xy_base_m=xy.tolist(),
                    minimum_prior_travel_m=distance,
                    minimum_history_s=seconds,
                    window_can_contain_direct_ground={str(w): seconds is not None and seconds <= w for w in (2, 5, 30)},
                )
            )
        sources = [
            Path(__file__),
            Path(__file__).with_name("local_terrain") / "observability.py",
            Path(__file__).with_name("archive_guard.py"),
            Path(__file__).with_name("revision_evidence.py"),
        ]
        source_hashes = {}
        for path in sources:
            target = output / "source" / path.relative_to(Path(__file__).parent)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
            source_hashes[str(path)] = sha(target)
        if any(sha(Path(p)) != digest for p, digest in hashes.items()):
            raise ValueError("observability diagnostic input changed")
        check_run(output)
        result = dict(
            acceptance=False,
            navigation_authority=False,
            source_failed=failed,
            model=__doc__,
            speed_m_s=speed,
            base_height_m=report["mean_base_height_m"],
            native_zenith_degrees=np.rad2deg(zenith).tolist(),
            wheels=results,
            input_sha256=hashes,
            source_sha256=source_hashes,
            warning="An optimistic cone bound, not actual ground coverage or authorization to extend evidence lifetime.",
        )
        with (output / "report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({k: result[k] for k in ("acceptance", "source_failed", "wheels")}, indent=2))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
