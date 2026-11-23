#!/usr/bin/env python3
"""Small-fixture frame audit and immutable local-evidence import (no motion).

This validates first-publication pose/cloud/graph agreement, not historical TF
reconstruction or terrain accuracy. Later graph poses remain append-only in the
database. Raw spinning scans are retained in the source bag, not re-deskewed here.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from astrobot_interfaces.msg import SlamRevision, SlamObservation
from archive_guard import check_run
from fixture_alignment import pose_matrix
from revision_evidence import RevisionEvidence

PREFIX = '/astrobot_0/slam_raw/'


def stamp(value):
    return value.sec * 1_000_000_000 + value.nanosec


def matrix(p):
    return pose_matrix([p.position.x, p.position.y, p.position.z],
                       [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def messages(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id='mcap'), ConverterOptions('', ''))
    while reader.has_next():
        yield reader.read_next()


def audit(run, max_frames=1000):
    check_run(run)
    if not 1 <= max_frames <= 10000:
        raise ValueError('audit is limited to 1..10000 frames; use a streaming mission evaluator for larger runs')
    bag = run / 'bag'
    counts, frames, by_stamp, poses, lidar_poses = Counter(), {}, {}, {}, {}
    identity, db = None, None
    pending, seen, paired, observed = {}, set(), set(), set()
    maximum_cloud_error = maximum_graph_error = 0.
    try:
        for topic, data, _ in messages(bag):
            counts[topic] += 1
            if topic == PREFIX + 'revisions':
                revision = deserialize_message(data, SlamRevision)
                if revision.header.frame_id != 'map':
                    raise ValueError('revision is not in map frame')
                current = (revision.session_id, revision.localization_epoch)
                if identity is None:
                    identity = current
                    db = RevisionEvidence(run / 'evidence.sqlite', *identity, create=True)
                if current != identity:
                    raise ValueError('multiple estimator sessions/epochs: split and validate the reset boundary first')
                updates = {}
                for f in revision.frames:
                    ts, extrinsic = stamp(f.measurement_stamp), matrix(f.imu_from_lidar)
                    if f.frame_id in updates:
                        raise ValueError('duplicate frame within revision')
                    if f.frame_id in frames:
                        old = frames[f.frame_id]
                        if old[0] != ts or not np.allclose(old[1], extrinsic, atol=1e-10, rtol=0):
                            raise ValueError('frame identity/extrinsic changed')
                    else:
                        if len(frames) >= max_frames or ts in by_stamp:
                            raise ValueError('fixture frame budget exceeded or ambiguous timestamp')
                        frames[f.frame_id] = (ts, extrinsic, matrix(f.map_from_imu), revision.revision)
                        by_stamp[ts] = f.frame_id
                    updates[f.frame_id] = matrix(f.map_from_imu) @ extrinsic
                db.apply_revision(revision.revision, revision.parent_revision, updates,
                                  snapshot=revision.full_snapshot,
                                  deskew_changed=any(f.deskew_recomputed for f in revision.frames))
                if revision.revision % 100 == 0:
                    check_run(run)
            elif topic in (PREFIX + 'pose_corrected', PREFIX + 'lidar_pose_corrected'):
                msg = deserialize_message(data, PoseStamped)
                target = poses if topic == PREFIX + 'pose_corrected' else lidar_poses
                ts = stamp(msg.header.stamp)
                if msg.header.frame_id != 'map' or ts in target or len(target) >= max_frames:
                    raise ValueError('invalid, duplicate or over-budget published pose')
                target[ts] = matrix(msg.pose)
        if db is None or not frames:
            raise ValueError('no revision evidence')
        for topic, data, _ in messages(bag):
            if topic == PREFIX + 'observations':
                observation = deserialize_message(data, SlamObservation)
                f, cloud = observation.frame, observation.points
                if (observation.session_id, observation.localization_epoch) != identity or f.frame_id not in frames:
                    raise ValueError('observation has unknown session/frame identity')
                ts, extrinsic, first_pose, first_revision = frames[f.frame_id]
                if (f.frame_id in observed or stamp(f.measurement_stamp) != ts or stamp(cloud.header.stamp) != ts
                        or observation.revision != first_revision
                        or not np.allclose(matrix(f.map_from_imu), first_pose, atol=1e-10, rtol=0)
                        or not np.allclose(matrix(f.imu_from_lidar), extrinsic, atol=1e-10, rtol=0)):
                    raise ValueError('observation disagrees with first graph revision')
                if cloud.width * cloud.height > 200000:
                    raise ValueError('per-cloud audit point budget exceeded')
                xyz = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'], skip_nans=False)
                if not len(xyz) or not np.isfinite(xyz).all():
                    raise ValueError('empty/nonfinite immutable observation')
                if cloud.header.frame_id == 'Imu_Sensor':
                    inverse = np.linalg.inv(extrinsic)
                    xyz = xyz @ inverse[:3, :3].T + inverse[:3, 3]
                elif cloud.header.frame_id != 'vlp16':
                    raise ValueError('observation is not sensor local')
                db.add_scan(f.frame_id, ts, 'vlp16', xyz.astype(np.float64))
                observed.add(f.frame_id)
                continue
            if topic not in (PREFIX + 'points_corrected', PREFIX + 'aligned_points_corrected'):
                continue
            msg = deserialize_message(data, PointCloud2)
            ts = stamp(msg.header.stamp)
            if ts not in by_stamp or ts not in poses or ts not in lidar_poses:
                raise ValueError('cloud lacks exact graph/IMU/LiDAR timestamp association')
            if msg.width * msg.height > 200000:
                raise ValueError('per-cloud audit point budget exceeded')
            xyz = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=False)
            if not np.isfinite(xyz).all():
                raise ValueError('nonfinite derived cloud evidence')
            slot = 'local' if topic == PREFIX + 'points_corrected' else 'aligned'
            if (ts, slot) in seen:
                raise ValueError('duplicate derived cloud timestamp')
            seen.add((ts, slot))
            pair = pending.setdefault(ts, {})
            pair[slot] = xyz
            fid = by_stamp[ts]
            _, extrinsic, first_pose, _ = frames[fid]
            if slot == 'local':
                if msg.header.frame_id == 'Imu_Sensor':
                    pair['map_from_local'] = poses[ts]
                elif msg.header.frame_id == 'vlp16':
                    pair['map_from_local'] = lidar_poses[ts]
                else:
                    raise ValueError('derived points are not in a supported immutable sensor frame')
                graph_error = float(np.max(np.abs(first_pose - poses[ts])))
                extrinsic_error = float(np.max(np.abs(poses[ts] @ extrinsic - lidar_poses[ts])))
                maximum_graph_error = max(maximum_graph_error, graph_error, extrinsic_error)
                if max(graph_error, extrinsic_error) > 1e-5:
                    raise ValueError('first graph revision and published pose/extrinsic disagree')
            elif msg.header.frame_id != 'map':
                raise ValueError('aligned cloud is not in map')
            if 'local' in pair and 'aligned' in pair:
                transform = pair['map_from_local']
                expected = pair['local'] @ transform[:3, :3].T + transform[:3, 3]
                if expected.shape != pair['aligned'].shape or not len(expected):
                    raise ValueError('empty or mismatched local/aligned clouds')
                error = float(np.max(np.abs(expected - pair['aligned'])))
                maximum_cloud_error = max(maximum_cloud_error, error)
                if error > 1e-4:
                    raise ValueError('aligned cloud coordinates disagree with published pose')
                paired.add(ts)
                del pending[ts]
            if len(pending) > 32:
                raise ValueError('unmatched cloud buffer budget exceeded')
        if observed != set(frames):
            raise ValueError(f'incomplete immutable evidence: {len(observed)} observations for {len(frames)} graph frames')
        if pending or len(paired) < 10 or paired != set(poses) or paired != set(lidar_poses):
            raise ValueError(f'incomplete visualization fixture: {len(paired)} cloud pairs for {len(poses)} poses')
        for required in ('/astrobot_0/lidar_0/pointcloud/raw', '/astrobot_0/imu', '/gt/base_link_pose', '/tf'):
            if not counts[required]:
                raise ValueError('missing required raw/reference topic: ' + required)
        check_run(run)
        return {'pass': True, 'session_epoch': identity, 'graph_frames': len(frames), 'cloud_pairs': len(paired),
                'immutable_observations': len(observed),
                'graph_frames_without_legacy_corrected_output': sorted(set(frames) - {by_stamp[ts] for ts in paired}),
                'final_revision': db.revision, 'topic_counts': dict(counts),
                'max_cloud_coordinate_error_m': maximum_cloud_error,
                'max_first_graph_pose_matrix_error': maximum_graph_error,
                'historical_tf_audit': 'NOT_RUN', 'raw_redeskew': 'NOT_IMPLEMENTED',
                'terrain_and_navigation_validation': 'NOT_RUN'}
    finally:
        if db is not None:
            db.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--max-frames', type=int, default=1000)
    args = parser.parse_args()
    # Reserve the result exclusively before creating derived artifacts.
    with (args.run / 'frame_audit.json').open('x') as output:
        try:
            result = audit(args.run, args.max_frames)
        except Exception as exc:
            result = {'pass': False, 'failure': str(exc), 'partial_evidence_retained': True}
        json.dump(result, output, indent=2)
    print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
