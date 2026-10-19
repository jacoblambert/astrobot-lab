#!/usr/bin/env python3
"""Audit live TF against same-time SE(3) poses, with a fixed independent GT anchor.

TF lacks a graph revision tag. This checks that each held correction equals a
causal, fresh authorized pose composition, not a unique publication generation.
Historical graph reconstruction is checked separately through revision evidence.
"""
import argparse
import bisect
import json
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from archive_guard import check_run
from audit_revision_fixture import messages, matrix, stamp
from fixture_alignment import ReferenceTrajectory, pose_matrix


def pose_row(msg):
    p, q = msg.pose.position, msg.pose.orientation
    return (stamp(msg.header.stamp) * 1e-9, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def correction_series(slam, odometry, imu_from_base):
    reference = ReferenceTrajectory(odometry)
    return [(ts, pose_matrix(xyz, q) @ imu_from_base @ np.linalg.inv(reference.at(ts)))
            for ts, xyz, q in slam]


def held_correction_errors(corrections, transforms, *, tolerance=.1, max_age=.5):
    times = [item[0] for item in corrections]
    errors = []
    for ts, actual in transforms:
        # The bridge dates its held correction at publication time + tolerance.
        publication = ts - tolerance
        start = bisect.bisect_left(times, publication - max_age - 1e-8)
        end = bisect.bisect_right(times, publication + 1e-8)
        if start == end:
            raise ValueError('TF has no causal fresh pose/odometry correction')
        errors.append(min(float(np.max(np.abs(actual - expected))) for _, expected in corrections[start:end]))
    if not errors:
        raise ValueError('no map->odom transforms')
    return errors


def audit(run, *, home_xyz=(20., 18., 0.)):
    check_run(run)
    odometry, slam, gt, transforms = [], [], [], []
    commands, nonzero = 0, 0
    edges = set()
    for topic, data, _ in messages(run / 'bag'):
        if topic == '/astrobot_0/odom':
            msg = deserialize_message(data, Odometry)
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
                raise ValueError('unexpected odometry frame/child')
            odometry.append(pose_row(PoseStamped(header=msg.header, pose=msg.pose.pose)))
        elif topic in ('/astrobot_0/slam/pose_corrected', '/gt/base_link_pose'):
            msg = deserialize_message(data, PoseStamped)
            expected_frame = 'map' if topic.endswith('pose_corrected') else 'gt_map'
            if msg.header.frame_id != expected_frame:
                raise ValueError('unexpected SLAM/GT pose frame')
            (slam if expected_frame == 'map' else gt).append(pose_row(msg))
        elif topic == '/tf':
            for msg in deserialize_message(data, TFMessage).transforms:
                edges.add((msg.header.frame_id, msg.child_frame_id))
                if msg.header.frame_id == 'map' and msg.child_frame_id == 'odom':
                    p, q = msg.transform.translation, msg.transform.rotation
                    transforms.append((stamp(msg.header.stamp) * 1e-9,
                        pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])))
        elif topic == '/astrobot_0/cmd_vel':
            msg = deserialize_message(data, Twist)
            values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
            commands += 1
            nonzero += int(not np.isfinite(values).all() or max(map(abs, values)) > 1e-6)
        if max(len(odometry), len(slam), len(gt), len(transforms)) > 20000:
            raise ValueError('small-fixture TF audit sample budget exceeded')
    if not slam or not commands:
        raise ValueError('missing SLAM or command evidence')
    if any('gt_map' in edge for edge in edges):
        raise ValueError('GT frame leaked into live TF')
    if ('odom', 'base_link') not in edges:
        raise ValueError('odom->base_link edge missing')
    base_from_imu = pose_matrix([.264, .017, .262], [0., 0., 1., 0.])
    imu_from_base = np.linalg.inv(base_from_imu)
    corrections = correction_series(slam, odometry, imu_from_base)
    errors = held_correction_errors(corrections, transforms)
    truth = ReferenceTrajectory(gt)
    position_errors, rotation_errors = [], []
    for ts, xyz, q in slam:
        predicted_base = pose_matrix(xyz, q) @ imu_from_base
        actual_base = truth.at(ts)
        actual_base[:3, 3] -= home_xyz
        position_errors.append(float(np.linalg.norm(predicted_base[:3, 3] - actual_base[:3, 3])))
        delta = predicted_base[:3, :3].T @ actual_base[:3, :3]
        rotation_errors.append(float(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1))))
    return {'pass': max(errors) < 1e-5 and nonzero == 0,
            'scope': 'stationary same-time full-SE3 wiring, not unique revision/TF generation or mission success',
            'tf_samples': len(errors), 'slam_samples': len(slam), 'commands': commands, 'nonzero_commands': nonzero,
            'max_tf_composition_matrix_error': max(errors), 'tf_edges': sorted(edges),
            'fixed_home_xyz': list(home_xyz), 'posthoc_alignment': False,
            'gt_base_position_error_m': {'p95': float(np.percentile(position_errors, 95)), 'max': max(position_errors)},
            'gt_base_rotation_error_rad': {'p95': float(np.percentile(rotation_errors, 95)), 'max': max(rotation_errors)},
            'gt_error_is_diagnostic_not_a_tuned_acceptance_gate': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    with (args.run / 'tf_audit.json').open('x') as out:
        try:
            result = audit(args.run)
        except Exception as exc:
            result = {'pass': False, 'failure': str(exc)}
        json.dump(result, out, indent=2)
    print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
