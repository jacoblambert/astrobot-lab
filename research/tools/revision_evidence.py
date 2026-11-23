"""Disk-backed, immutable LOCAL scan evidence and append-only SE(3) revisions.

This is a reconstruction reference, not a live terrain mapper. Input scans must
already be in the specified sensor frame at their measurement time. Raw spinning
scans require validated deskewing upstream; a pose update does not do that job.
"""
import hashlib
import io
import json
import math
from pathlib import Path
import sqlite3

import numpy as np


def array_bytes(value):
    stream = io.BytesIO()
    np.save(stream, value, allow_pickle=False)
    return stream.getvalue()


def array_from_bytes(value):
    return np.load(io.BytesIO(value), allow_pickle=False)


def validated_transform(value):
    value = np.asarray(value, dtype=np.float64)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.allclose(value[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or abs(np.linalg.det(value[:3, :3])-1) > 1e-8):
        raise ValueError("pose is not finite rigid SE(3)")
    return value


class RevisionEvidence:
    """One database per estimator session/epoch; callers own archive quotas.

    Transactions make revision acceptance atomic. Duplicate identical messages
    are idempotent; conflicting identities or missing deltas are rejected.
    Reopening requires explicit session/epoch agreement. No world-space point
    cache is authoritative and no old row is replaced by a correction.
    """
    def __init__(self, path, session, epoch, *, create=False):
        path = Path(path)
        if not session or epoch < 0:
            raise ValueError("invalid session/epoch")
        if create:
            # Exclusive creation avoids silently adopting another run's data.
            with path.open("xb"):
                pass
        elif not path.is_file():
            raise FileNotFoundError(path)
        self.db = sqlite3.connect(str(path))
        self.db.execute("PRAGMA foreign_keys=ON")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.execute("PRAGMA cache_size=-8192")
        if create:
            self.db.executescript("""
                CREATE TABLE metadata(session TEXT, epoch INTEGER);
                CREATE TABLE scans(frame INTEGER PRIMARY KEY, stamp_ns INTEGER NOT NULL,
                    sensor_frame TEXT NOT NULL, points BLOB NOT NULL, digest TEXT NOT NULL);
                CREATE TABLE revisions(revision INTEGER PRIMARY KEY, parent INTEGER NOT NULL,
                    snapshot INTEGER NOT NULL, digest TEXT NOT NULL);
                CREATE TABLE poses(frame INTEGER, revision INTEGER REFERENCES revisions(revision),
                    transform BLOB NOT NULL, PRIMARY KEY(frame,revision));
                CREATE INDEX poses_revision ON poses(revision);
            """)
            with self.db:
                self.db.execute("INSERT INTO metadata VALUES (?,?)", (session, epoch))
        if self.db.execute("SELECT session,epoch FROM metadata").fetchone() != (session, epoch):
            self.db.close()
            raise ValueError("session/epoch mismatch")

    def close(self):
        self.db.close()

    @property
    def revision(self):
        return self.db.execute("SELECT COALESCE(MAX(revision),0) FROM revisions").fetchone()[0]

    def add_scan(self, frame, stamp_ns, sensor_frame, points):
        points = np.asarray(points)
        if (frame < 0 or stamp_ns < 0 or not sensor_frame or points.ndim != 2
                or points.shape[1] != 3 or points.dtype.kind != "f"):
            raise ValueError("invalid local scan")
        blob = array_bytes(points)
        digest = hashlib.sha256(blob).hexdigest()
        old = self.db.execute("SELECT stamp_ns,sensor_frame,digest FROM scans WHERE frame=?", (frame,)).fetchone()
        if old is not None:
            if old != (stamp_ns, sensor_frame, digest):
                raise ValueError("conflicting immutable scan identity")
            return False
        with self.db:
            self.db.execute("INSERT INTO scans VALUES (?,?,?,?,?)", (frame, stamp_ns, sensor_frame, blob, digest))
        return True

    def apply_revision(self, revision, parent, poses, *, snapshot=False, deskew_changed=False):
        if deskew_changed:
            raise ValueError("re-deskewed evidence required; rigid placement is insufficient")
        if revision <= 0 or parent < 0 or parent >= revision:
            raise ValueError("invalid revision sequence")
        rows = []
        for frame, pose in sorted(poses.items()):
            if frame < 0:
                raise ValueError("invalid frame identity")
            rows.append((frame, array_bytes(validated_transform(pose))))
        h = hashlib.sha256(json.dumps([revision, parent, snapshot]).encode())
        for frame, blob in rows:
            h.update(str(frame).encode()+b":"+blob)
        digest = h.hexdigest()
        old = self.db.execute("SELECT digest FROM revisions WHERE revision=?", (revision,)).fetchone()
        if old:
            if old[0] != digest:
                raise ValueError("conflicting revision identity")
            return False
        latest = self.revision
        if revision <= latest or (not snapshot and (parent != latest or revision != latest+1)):
            raise ValueError("revision gap/order mismatch: request authoritative snapshot")
        if snapshot:
            known = {row[0] for row in self.db.execute("SELECT DISTINCT frame FROM poses")}
            if not known.issubset(poses):
                raise ValueError("snapshot omitted known frame identities")
        with self.db:
            self.db.execute("INSERT INTO revisions VALUES (?,?,?,?)", (revision, parent, snapshot, digest))
            self.db.executemany("INSERT INTO poses VALUES (?,?,?)", [(frame, revision, blob) for frame, blob in rows])
        return True

    def scans_at(self, revision, *, as_of_ns, not_before_ns=0):
        """Stream reprojected scans without accumulating the mission in RAM."""
        if self.db.execute("SELECT 1 FROM revisions WHERE revision=?", (revision,)).fetchone() is None:
            raise ValueError("unknown revision")
        query = """SELECT s.frame,s.stamp_ns,s.points,p.transform FROM scans s
            LEFT JOIN poses p ON p.frame=s.frame AND p.revision=(
                SELECT MAX(q.revision) FROM poses q WHERE q.frame=s.frame AND q.revision<=?)
            WHERE s.stamp_ns<=? AND s.stamp_ns>=? ORDER BY s.stamp_ns,s.frame"""
        for frame, stamp_ns, points_blob, pose_blob in self.db.execute(query, (revision, as_of_ns, not_before_ns)):
            if pose_blob is None:
                raise ValueError(f"missing pose for scan {frame}")
            local, pose = array_from_bytes(points_blob), array_from_bytes(pose_blob)
            points = local @ pose[:3, :3].T + pose[:3, 3]
            yield frame, stamp_ns, points, pose[:3, 3].copy()

    def height_evidence(self, revision, *, as_of_ns, resolution=.1, lifetime_sec=30.):
        """Deterministic reference XY bins, NOT ground classification/costmap.

        Rebuild from scratch is the oracle for incremental implementations. Each
        value includes original observation time; revision time never refreshes it.
        """
        if not math.isfinite(resolution) or resolution <= 0 or not math.isfinite(lifetime_sec) or lifetime_sec < 0:
            raise ValueError("invalid evidence geometry/age")
        cells = {}
        for frame, stamp_ns, points, _ in self.scans_at(revision, as_of_ns=as_of_ns,
                not_before_ns=max(0, math.ceil(as_of_ns-lifetime_sec*1e9))):
            for x, y, z in points:
                if not np.isfinite([x, y, z]).all():
                    continue
                key = (math.floor(x/resolution), math.floor(y/resolution))
                cells.setdefault(key, []).append((float(z), stamp_ns, frame))
        return {key: sorted(value) for key, value in sorted(cells.items())}
