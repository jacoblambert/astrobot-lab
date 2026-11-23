"""Bounded exact range/transform cache; no spatial cropping or approximation."""

from collections import OrderedDict

import numpy as np

from raw_return_filter import valid_range_mask
from revision_evidence import validated_transform


class ReturnTransformCache:
    def __init__(self, minimum, maximum, *, max_bytes=64 * 1024**2, max_entries=2048, retain_transforms=False):
        if type(max_bytes) is not int or max_bytes < 0 or type(max_entries) is not int or max_entries <= 0:
            raise ValueError("invalid exact cache budget")
        valid_range_mask(np.empty((0, 3)), minimum, maximum)
        self.minimum, self.maximum = minimum, maximum
        self.max_bytes, self.max_entries = max_bytes, max_entries
        if type(retain_transforms) is not bool:
            raise ValueError("invalid transform retention switch")
        self.retain_transforms = retain_transforms
        self.entries = OrderedDict()
        self.bytes = self.hits = self.transforms = self.filters = 0

    def clear(self):
        self.entries.clear()
        self.bytes = 0

    def prune(self, active_sequences):
        keep = set(active_sequences)
        for key in list(self.entries):
            if key not in keep:
                self._remove(key)

    def _remove(self, key):
        _, _, selected, _, world = self.entries.pop(key)
        self.bytes -= selected.nbytes + (0 if world is None else world.nbytes)

    def transform(self, sequence, stamp, raw, pose):
        pose = validated_transform(pose)
        if raw.flags.writeable:
            raise ValueError("exact cache requires owned immutable native arrays")
        entry = self.entries.get(sequence)
        if entry is not None:
            old_stamp, old_raw, selected, old_pose, world = entry
            if old_stamp != stamp or old_raw is not raw:
                raise ValueError("native cache identity changed")
            if world is not None and np.array_equal(old_pose, pose):
                self.hits += 1
                return world
        else:
            keep, _ = valid_range_mask(raw, self.minimum, self.maximum)
            selected = raw[keep].astype(np.float64)
            selected.flags.writeable = False
            self.filters += 1
        world = selected @ pose[:3, :3].T + pose[:3, 3]
        world.flags.writeable = False
        self.transforms += 1
        required = selected.nbytes + (world.nbytes if self.retain_transforms else 0)
        if entry is not None:
            self._remove(sequence)
        if required <= self.max_bytes:
            while self.entries and (self.bytes + required > self.max_bytes or len(self.entries) >= self.max_entries):
                self._remove(next(iter(self.entries)))
            self.entries[sequence] = (
                stamp,
                raw,
                selected,
                pose.copy() if self.retain_transforms else None,
                world if self.retain_transforms else None,
            )
            self.bytes += required
        return world
