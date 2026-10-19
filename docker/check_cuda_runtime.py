#!/usr/bin/env python3
"""Bounded CUDA initialization/allocation probe; NVML visibility is insufficient.

Creates and destroys only this process's context and four-byte allocation. Never
repairs drivers or resets hardware. Passing is a prerequisite, not an Isaac,
Vulkan, sensor-geometry or navigation acceptance test.
"""
import ctypes
import json
import subprocess
import sys


def probe_cuda(load_library=ctypes.CDLL):
    driver = load_library("libcuda.so.1")

    def call(name, types, *args):
        function = getattr(driver, name)
        function.argtypes = types
        function.restype = ctypes.c_int
        code = function(*args)
        if code:
            raise RuntimeError(f"{name} failed with CUDA error {code}")

    integer = ctypes.c_int
    pointer = ctypes.c_void_p
    address = ctypes.c_uint64
    call("cuInit", [ctypes.c_uint], 0)
    count = integer()
    call("cuDeviceGetCount", [ctypes.POINTER(integer)], ctypes.byref(count))
    if count.value < 1:
        raise RuntimeError("CUDA reports no accessible devices")
    device = integer()
    call("cuDeviceGet", [ctypes.POINTER(integer), integer], ctypes.byref(device), 0)
    context = pointer()
    call("cuCtxCreate_v2", [ctypes.POINTER(pointer), ctypes.c_uint, integer], ctypes.byref(context), 0, device)
    try:
        allocation = address()
        call("cuMemAlloc_v2", [ctypes.POINTER(address), ctypes.c_size_t], ctypes.byref(allocation), 4)
        try:
            call("cuCtxSynchronize", [])
        finally:
            call("cuMemFree_v2", [address], allocation)
    finally:
        call("cuCtxDestroy_v2", [pointer], context)
    return dict(passed=True, devices=count.value, context_and_allocation_checked=True,
                isaac_acceptance=False)


def bounded_probe(run=subprocess.run):
    try:
        result = run([sys.executable, __file__, "--worker"], capture_output=True, text=True, timeout=15)
    except subprocess.TimeoutExpired:
        return dict(passed=False, error="CUDA probe exceeded 15 seconds; no repair attempted")
    if result.returncode:
        return dict(passed=False, error=result.stderr.strip() or result.stdout.strip() or
                    f"CUDA worker exited {result.returncode}")
    try:
        report = json.loads(result.stdout)
        if report.get("passed") is not True or report.get("context_and_allocation_checked") is not True:
            raise ValueError("incomplete CUDA probe")
        return report
    except (ValueError, AttributeError):
        return dict(passed=False, error="invalid CUDA worker report")


def main():
    if sys.argv[1:] == ["--worker"]:
        try:
            print(json.dumps(probe_cuda()))
        except Exception as exc:
            print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
            return 1
        return 0
    report = bounded_probe()
    print(json.dumps(report), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
