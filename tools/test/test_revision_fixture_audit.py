"""Actual ROS serialization; replay reader is deterministic and has no live DDS."""
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import serialize_message
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from astrobot_interfaces.msg import SlamFramePose, SlamRevision, SlamObservation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from archive_guard import allocate_run
from audit_revision_fixture import PREFIX, audit
from revision_evidence import RevisionEvidence


def fixture(*, corrupt_cloud=False, revision_gap=False):
    revision = SlamRevision(session_id='test', localization_epoch=1, revision=1, parent_revision=0)
    revision.header.frame_id = 'map'
    if revision_gap:
        revision.revision, revision.parent_revision = 3, 2
    records = []
    for i in range(10):
        pose = PoseStamped(header=Header(frame_id='map'))
        pose.header.stamp.sec = i + 1
        pose.pose.position.x = float(i)
        pose.pose.orientation.z, pose.pose.orientation.w = np.sin(.2), np.cos(.2)
        f = SlamFramePose(frame_id=i, submap_id=-1, measurement_stamp=pose.header.stamp,
                          map_from_imu=pose.pose)
        f.imu_from_lidar.orientation.w = 1.
        f.imu_from_lidar.position.z = .2
        revision.frames.append(f)
        lidar = PoseStamped(header=pose.header)
        lidar.pose.position.x = float(i)
        lidar.pose.position.z = .2
        lidar.pose.orientation = pose.pose.orientation
        local = np.array([[.025, .025, 0.], [.025, .025, .08]], dtype=np.float32)
        c, s = np.cos(.4), np.sin(.4)
        aligned = local @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T + [i, 0, 0]
        if corrupt_cloud:
            aligned[:, 0] += 1.
        local_header = Header(stamp=pose.header.stamp, frame_id='Imu_Sensor')
        observation = SlamObservation(session_id='test', localization_epoch=1, revision=1,
            frame=f, points=create_cloud_xyz32(local_header, local), preprocessing_lossy=True)
        records.append((PREFIX + 'observations', serialize_message(observation), i))
        for suffix, msg in (
            ('pose_corrected', pose), ('lidar_pose_corrected', lidar),
            ('points_corrected', create_cloud_xyz32(local_header, local)),
            ('aligned_points_corrected', create_cloud_xyz32(pose.header, aligned))):
            records.append((PREFIX + suffix, serialize_message(msg), i))
    records.insert(0, (PREFIX + 'revisions', serialize_message(revision), 0))
    for topic in ('/astrobot_0/lidar_0/pointcloud/raw', '/astrobot_0/imu', '/gt/base_link_pose', '/tf'):
        records.append((topic, b'', 0))
    return records


def test_import_preserves_local_evidence_and_ray_origin(tmp_path):
    run = allocate_run(tmp_path, limit=10**7, reserve=0, external=False)
    with patch('audit_revision_fixture.messages', side_effect=lambda _: iter(fixture())):
        result = audit(run)
    assert result['pass'] and result['cloud_pairs'] == 10
    db = RevisionEvidence(run / 'evidence.sqlite', 'test', 1)
    _, ts, points, origin = next(db.scans_at(1, as_of_ns=10**10))
    np.testing.assert_allclose(origin, [0, 0, .2])
    np.testing.assert_allclose(points[:, 2], [0, .08], atol=1e-8)
    assert ts == 10**9
    db.close()


@pytest.mark.parametrize('options,reason', [({'corrupt_cloud': True}, 'coordinates disagree'),
                                           ({'revision_gap': True}, 'gap')])
def test_audit_rejects_wrong_frame_and_missing_revision(tmp_path, options, reason):
    run = allocate_run(tmp_path, limit=10**7, reserve=0, external=False)
    with patch('audit_revision_fixture.messages', side_effect=lambda _: iter(fixture(**options))):
        with pytest.raises(ValueError, match=reason):
            audit(run)
    assert (run / 'evidence.sqlite').exists()  # Failed evidence is retained.
