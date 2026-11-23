"""Bounded, stationary-only GC ownership diagnostic; no GC policy changes."""

import gc
import hashlib
import heapq
from pathlib import Path
import sys
import time
from collections import Counter


def type_name(value):
    kind = type(value)
    return str(kind.__module__) + "." + str(kind.__name__)


def profile_heap(lease_sequence, motion_sent, capture_started):
    if lease_sequence != -1 or motion_sent or capture_started is not None or not gc.isenabled():
        raise ValueError("heap profiling requires pre-lease stationary initialization and enabled GC")
    started = time.monotonic()
    collections = []
    for _ in range(3):
        before = time.monotonic()
        collected = gc.collect(2)
        collections.append(dict(wall_s=time.monotonic() - before, collected=collected))
    objects = gc.get_objects()
    totals, counts, frames = Counter(), Counter(), Counter()
    largest = []
    for value in objects:
        name = type_name(value)
        size = sys.getsizeof(value, 0)
        totals[name] += size
        counts[name] += 1
        if type(value) is type(sys._getframe()):
            frames[value.f_code.co_filename + ":" + value.f_code.co_name] += 1
        if type(value) in (list, tuple, dict, set):
            entry = (size, id(value), value)
            if len(largest) < 12:
                heapq.heappush(largest, entry)
            elif entry[:2] > largest[0][:2]:
                heapq.heapreplace(largest, entry)
    containers = []
    for size, identity, value in sorted(largest, key=lambda row: row[:2], reverse=True):
        owners = []
        for parent in gc.get_referrers(value):
            if parent is objects or parent is largest:
                continue
            row = dict(type=type_name(parent))
            if type(parent) is dict:
                row["keys_referring_to_object"] = [
                    key for key, child in parent.items() if child is value and type(key) is str
                ][:12]
                module = parent.get("__name__")
                if type(module) is str:
                    row["module"] = module
            owners.append(row)
            if len(owners) == 12:
                break
        containers.append(dict(type=type_name(value), length=len(value), shallow_bytes=size, owners=owners))
    return dict(
        diagnostic_only=True, acceptance=False, motion_commands_sent=False,
        tracked_objects=len(objects), shallow_bytes_by_type=totals.most_common(25),
        tracked_counts=counts.most_common(25), retained_frames=frames.most_common(25),
        largest_containers=containers, wall_s=time.monotonic() - started,
        gc_enabled=gc.isenabled(), thresholds=list(gc.get_threshold()),
        stationary_full_collections=collections,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    )
