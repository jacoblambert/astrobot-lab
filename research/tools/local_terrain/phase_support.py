"""Sub-cell support from independently approved shifted grids; diagnostic only.

No interpolation, dilation, optimistic unknown filling, or raw-map edits. Each
phase uses the same classifier; its hazards veto a sweep in every other phase.
This is not a substitute for native sensing, age/uncertainty or runtime gates.
"""

from types import SimpleNamespace

import numpy as np
from shapely import box, union_all

from local_terrain.contact_query import PreparedMotion

PHASES = ((0.0, 0.0), (0.05, 0.0), (0.0, 0.05), (0.05, 0.05))


class PhaseSupport:
    def __init__(self, baseline, grids, record):
        if len(grids) != 3:
            raise ValueError("exactly three extra declared phases required")
        self.origin = np.asarray(record["origin"])
        self.resolution = record["resolution"]
        if self.resolution != 0.1 or not np.array_equal(baseline.origin, self.origin):
            raise ValueError("phase baseline grid mismatch")
        self.baseline, self.grids = baseline, []
        self.cache = {}
        for (layers, phase_record), offset in zip(grids, PHASES[1:]):
            if layers.shape != (60, 60, 6) or not np.isin(layers[:, :, 0], (-1, 0, 20, 100)).all():
                raise ValueError("complete declared phase grid required")
            for key in ("session", "epoch", "revision", "evidence_ns", "publication_ns", "resolution"):
                if phase_record[key] != record[key]:
                    raise ValueError("phase generation/time mismatch")
            if not np.array_equal(phase_record["local_from_map"], record["local_from_map"]):
                raise ValueError("phase coordinate-frame mismatch")
            if not np.allclose(np.asarray(phase_record["origin"]) - self.origin, offset, atol=1e-12, rtol=0):
                raise ValueError("undeclared grid phase")
            self.grids.append((layers, phase_record))
        self.index = SimpleNamespace(backend="geos", origin=self.origin, resolution=self.resolution, unions=self)

    def get(self, ij):
        """Exact geometric coverage for one baseline cell; cached per publication."""
        if ij in self.cache:
            return self.cache[ij]
        if not (0 <= ij[0] < 60 and 0 <= ij[1] < 60) or len(self.cache) >= 3600:
            raise ValueError("phase support query budget")
        r = self.resolution
        low = self.origin + np.asarray(ij) * r
        high = low + r
        covers = []
        original = self.baseline.unions.get(ij)
        if original is not None:
            covers.append(original)
        for layers, record in self.grids:
            origin = np.asarray(record["origin"])
            first = np.floor((low - origin) / r).astype(int)
            last = np.floor((high - origin) / r).astype(int)
            for y in range(max(0, first[1]), min(59, last[1]) + 1):
                for x in range(max(0, first[0]), min(59, last[0]) + 1):
                    if layers[y, x, 0] in (0, 20):
                        p = origin + [x * r, y * r]
                        covers.append(box(*p, *(p + r)))
        self.cache[ij] = union_all(covers) if covers else None
        return self.cache[ij]

    def query(self, layers, record, poses, patches, body):
        baseline = PreparedMotion(layers, record, poses, patches, body)
        if baseline.terminal is not None:
            return baseline.terminal.copy()
        if baseline.conflicts:
            return dict(state="unknown", reason="baseline_hazard_conflict")
        for shifted, shifted_record in self.grids:
            check = PreparedMotion(shifted, shifted_record, poses, patches, body)
            if check.terminal is not None:
                return dict(check.terminal, phase_veto=True)
            if check.conflicts:
                return dict(state="unknown", reason="phase_hazard_conflict")
        return baseline.evaluate(self.index)
