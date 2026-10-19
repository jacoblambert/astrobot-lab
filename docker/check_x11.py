#!/usr/bin/env python3
"""Verify container X11 authorization before Isaac can silently lose its GUI.

Opens/closes an X connection only. Does not change access control, display state
or expose cookies. The host's xdpyinfo alone cannot validate container access.
"""

import ctypes
import json


def probe_x11(load_library=ctypes.CDLL):
    x11 = load_library("libX11.so.6")
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.restype = ctypes.c_int
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError("Cannot open DISPLAY from container; check XAUTHORITY and XAUTHLOCALHOSTNAME")
    x11.XCloseDisplay(display)
    return dict(passed=True, container_x11_authorized=True, isaac_acceptance=False)


if __name__ == "__main__":
    try:
        print(json.dumps(probe_x11()))
    except Exception as exc:
        print(json.dumps(dict(passed=False, error=str(exc))))
        raise SystemExit(1)
