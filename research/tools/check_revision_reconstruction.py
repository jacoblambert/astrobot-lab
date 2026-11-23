#!/usr/bin/env python3
"""Nonuniform SE(3) correction check on archived REAL local scan evidence.

Compares a revised history against fresh final-pose reconstruction. Height bins
are a reference oracle, not a terrain classifier or real-time costmap. Restrict
to a declared recent measurement-time window to bound reference RAM and runtime.
"""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import sqlite3
import time

import numpy as np
from scipy.spatial.transform import Rotation
from archive_guard import check_run
from revision_evidence import RevisionEvidence, array_from_bytes


def digest(cells):
    return hashlib.sha256(repr(cells).encode()).hexdigest()


def check(run, window=1.):
    if not .1 <= window <= 5:
        raise ValueError('reference window must be .1..5 seconds')
    check_run(run)
    source = sqlite3.connect(f'file:{run / "evidence.sqlite"}?mode=ro', uri=True)
    session, epoch = source.execute('SELECT session,epoch FROM metadata').fetchone()
    as_of = source.execute('SELECT MAX(stamp_ns) FROM scans').fetchone()[0]
    count = source.execute('SELECT COUNT(*) FROM scans WHERE stamp_ns>=?', (as_of-int(window*1e9),)).fetchone()[0]
    if not 2 <= count <= 100:
        raise ValueError('reference window needs 2..100 scans')
    revised = RevisionEvidence(run / 'reconstruction_history.sqlite', session, epoch, create=True)
    fresh = RevisionEvidence(run / 'reconstruction_fresh.sqlite', session, epoch, create=True)
    try:
        original, final, point_count = {}, {}, 0
        rows = source.execute('SELECT frame,stamp_ns,sensor_frame,points FROM scans WHERE stamp_ns>=? ORDER BY stamp_ns,frame',
                              (as_of-int(window*1e9),))
        for index, (fid, ts, sensor, blob) in enumerate(rows):
            points = array_from_bytes(blob)
            point_count += len(points)
            for db in (revised, fresh):
                db.add_scan(fid, ts, sensor, points)
            original[fid] = array_from_bytes(source.execute(
                'SELECT transform FROM poses WHERE frame=? ORDER BY revision DESC LIMIT 1', (fid,)).fetchone()[0])
            delta = np.eye(4)
            # Two independent region corrections, including roll and vertical displacement.
            delta[:3, :3] = Rotation.from_euler('xyz', [.03, -.02, .08] if index < count//2 else [-.02, .01, -.1]).as_matrix()
            delta[:3, 3] = [.35, .2, .04] if index < count//2 else [-.3, .4, -.03]
            final[fid] = delta @ original[fid]
        revised.apply_revision(1, 0, original)
        before = revised.height_evidence(1, as_of_ns=as_of, lifetime_sec=window)
        revised.apply_revision(2, 1, final)
        fresh.apply_revision(1, 0, final)
        start = time.perf_counter()
        actual = revised.height_evidence(2, as_of_ns=as_of, lifetime_sec=window)
        revised_sec = time.perf_counter() - start
        start = time.perf_counter()
        expected = fresh.height_evidence(1, as_of_ns=as_of, lifetime_sec=window)
        fresh_sec = time.perf_counter() - start
        expired = revised.height_evidence(2, as_of_ns=as_of+int((window+1)*1e9), lifetime_sec=window)
        check_run(run)
        return {'pass': actual == expected and before != actual and not expired,
                'fixture_type': 'real local scans with synthetic nonuniform SE3 revisions',
                'measurement_window_sec': window, 'frames': count, 'points': point_count,
                'revised_sha256': digest(actual), 'fresh_sha256': digest(expected),
                'old_only_cells_removed': len(set(before)-set(actual)), 'new_only_cells_added': len(set(actual)-set(before)),
                'observation_ages_preserved': not expired,
                'reference_reconstruction_sec': revised_sec, 'reference_fresh_sec': fresh_sec,
                'process_peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                'not_a_realtime_mapper_or_actual_loop_closure_benchmark': True}
    finally:
        source.close()
        revised.close()
        fresh.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--window-seconds', type=float, default=1.)
    args = parser.parse_args()
    with (args.run / 'reconstruction_check.json').open('x') as out:
        try:
            result = check(args.run, args.window_seconds)
        except Exception as exc:
            result = {'pass': False, 'failure': str(exc)}
        json.dump(result, out, indent=2)
    print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
