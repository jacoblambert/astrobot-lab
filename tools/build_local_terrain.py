#!/usr/bin/env python3
"""Build an immutable, opt-in local terrain library and C++ fixtures on archive."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_build(library, archive):
    library, archive = Path(library).resolve(strict=True), Path(archive).resolve(strict=True)
    if not library.is_relative_to(archive):
        raise ValueError("candidate library must reside on declared archive")
    manifest_path = library.parent / "build.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema") != 1 or manifest.get("library") != library.name:
        raise ValueError("invalid local library build manifest")
    if digest(library) != manifest["library_sha256"]:
        raise ValueError("candidate library changed")
    hashes = {str(library): digest(library), str(manifest_path): digest(manifest_path)}
    for relative, expected in manifest["source_sha256"].items():
        path = (library.parent / relative).resolve(strict=True)
        if not path.is_relative_to(library.parent) or digest(path) != expected:
            raise ValueError("candidate build source changed or escaped archive")
        hashes[str(path)] = expected
    return hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--patch-normal-cache", action="store_true")
    parser.add_argument("--sparse-native-patch", action="store_true")
    parser.add_argument("--defer-native", action="store_true")
    parser.add_argument("--serial", action="store_true")
    parser.add_argument("--rolling-grid", action="store_true", help="opt-in integer-cell translation API")
    parser.add_argument("--diagnostics", action="store_true", help="rejection telemetry; not compute acceptance")
    parser.add_argument("--pointwise-boundary", action="store_true", help="experimental actual-return boundary test")
    args = parser.parse_args()
    if args.defer_native and not args.patch_normal_cache:
        parser.error("--defer-native requires --patch-normal-cache")
    if args.defer_native and args.diagnostics:
        parser.error("--diagnostics uses immediate native classification; omit --defer-native")
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    source = Path(__file__).resolve().parents[1] / "astrobot-lab/src/slam_mapping"
    paths = [
        *sorted((source / "include/slam_mapping").glob("*.hpp")),
        source / "src/layered_terrain.cpp",
        source / "src/layered_terrain_c.cpp",
        source / "test/test_layered_terrain.cpp",
        source / "test/test_observation_moments.cpp",
        *sorted((source / "test").glob("*fixture.hpp")),
    ]
    hashes = {}
    for path in paths:
        target = output / "source" / path.relative_to(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        hashes[str(target.relative_to(output))] = digest(target)
    shutil.copyfile(__file__, output / "build_local_terrain.py")
    hashes["build_local_terrain.py"] = digest(output / "build_local_terrain.py")
    flags = [
        "-std=c++17",
        "-O3",
        "-DNDEBUG",
        "-I/usr/include/eigen3",
        "-I" + str(output / "source/include"),
        "-DTERRAIN_OBSERVATIONS",
        "-DTERRAIN_TWO_PATH",
        "-DTERRAIN_RESOLVED_BOUNDARY",
    ]
    if not args.serial:
        flags.append("-fopenmp")
    if args.patch_normal_cache:
        flags.append("-DTERRAIN_PATCH_NORMAL_CACHE")
    if args.sparse_native_patch:
        flags.append("-DTERRAIN_SPARSE_NATIVE_PATCH")
    if args.defer_native:
        flags.append("-DTERRAIN_DEFER_NATIVE")
    if args.rolling_grid:
        flags.append("-DTERRAIN_ROLLING_GRID")
    if args.diagnostics:
        flags.append("-DTERRAIN_DIAGNOSTICS")
    if args.pointwise_boundary:
        flags.append("-DTERRAIN_POINTWISE_BOUNDARY")
    base = ["g++", *flags]
    commands = [
        base
        + [
            "-fPIC",
            "-shared",
            str(output / "source/src/layered_terrain.cpp"),
            str(output / "source/src/layered_terrain_c.cpp"),
            "-o",
            str(output / "liblocal_terrain.so"),
        ]
    ]
    # Assertions MUST remain enabled in tests, even though the library is release.
    for name in ("test_layered_terrain", "test_observation_moments"):
        commands.append(
            [v for v in base if v != "-DNDEBUG"]
            + [str(output / f"source/test/{name}.cpp")]
            + ([str(output / "source/src/layered_terrain.cpp")] if name == "test_layered_terrain" else [])
            + ["-o", str(output / name)]
        )
    try:
        compiler = subprocess.check_output(["g++", "--version"], text=True)
        with (output / "build.log").open("x") as log:
            for command in commands:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=300)
        check_run(output)
        manifest = dict(
            schema=1,
            library="liblocal_terrain.so",
            library_sha256=digest(output / "liblocal_terrain.so"),
            source_sha256=hashes,
            compiler=compiler,
            commands=commands,
            acceptance=False,
            patch_normal_cache=args.patch_normal_cache,
            sparse_native_patch=args.sparse_native_patch,
            defer_native=args.defer_native,
            serial=args.serial,
            rolling_grid=args.rolling_grid,
            diagnostics=args.diagnostics,
            pointwise_boundary=args.pointwise_boundary,
        )
        with (output / "build.json").open("x") as stream:
            json.dump(manifest, stream, indent=2)
        print(json.dumps({k: v for k, v in manifest.items() if k not in ("commands", "source_sha256", "compiler")}))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False, commands=commands), stream)
        raise


if __name__ == "__main__":
    main()
