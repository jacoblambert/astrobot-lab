#!/usr/bin/env python3
"""Record raw/revision evidence to the external archive. Sends NO motion commands.

Run in the sourced ROS environment with the external drive mounted at /archive,
or on a ROS host using its full external path. Collection success is NOT a map,
frame, reference-validity or autonomy success verdict. Run validation separately.
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time
from archive_guard import ARCHIVE_ROOT, allocate_run, check_run

TOPICS = [
    '/clock', '/tf', '/tf_static', '/gt/base_link_pose', '/gt/map',
    '/astrobot_0/odom', '/astrobot_0/imu', '/astrobot_0/lidar_0/pointcloud',
    '/astrobot_0/lidar_0/pointcloud/raw',
    '/astrobot_0/lidar_0/pointcloud/filtered', '/rtk/pose',
    '/astrobot_0/localization/absolute_pose', '/astrobot_0/localization/status',
    '/astrobot_0/slam_raw/revisions', '/astrobot_0/slam_raw/pose_corrected',
    '/astrobot_0/slam_raw/observations',
    '/astrobot_0/slam_raw/points_corrected',
    '/astrobot_0/slam_raw/lidar_pose_corrected', '/astrobot_0/slam_raw/aligned_points_corrected',
    '/astrobot_0/slam/pose_corrected', '/astrobot_0/slam/lidar_pose_corrected',
    '/astrobot_0/slam/aligned_points_corrected', '/astrobot_0/slam/bev_costmap',
    '/astrobot_0/cmd_vel',
]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-root',type=Path,default=Path('/archive') if Path('/archive').is_dir() else ARCHIVE_ROOT)
    parser.add_argument('--wall-seconds',type=float,default=60)
    args=parser.parse_args()
    if not 1 <= args.wall_seconds <= 7200:
        parser.error('wall-seconds must be between 1 and 7200')
    run=allocate_run(args.archive_root)
    print(run, flush=True)
    command=['ros2','bag','record','--storage','mcap','--max-bag-size','1073741824',
             '--max-cache-size','67108864','-o',str(run/'bag'),*TOPICS]
    with (run/'record.log').open('x') as log:
        process=subprocess.Popen(command,stdout=log,stderr=log)
        start=time.monotonic(); failure=None
        try:
            while time.monotonic()-start < args.wall_seconds:
                if process.poll() is not None:
                    raise RuntimeError('recorder exited before collection deadline')
                check_run(run)
                time.sleep(.5)
        except (Exception,KeyboardInterrupt) as exc:
            failure=str(exc) or 'interrupted'
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(); failure='recorder failed to finalize'
    if process.returncode != 0:
        failure=failure or f'recorder exit {process.returncode}'
    if not (run/'bag/metadata.yaml').exists():
        failure=failure or 'bag metadata missing'
    try:
        check_run(run)
    except Exception as exc:
        failure=failure or str(exc)
    summary={'collection_completed':failure is None,'failure':failure,'command':command,
             'wall_seconds':time.monotonic()-start,'validation':'NOT_RUN',
             'note':'No motion commands sent; topic completeness/reference integrity must be checked separately.'}
    with (run/'collection.json').open('x') as out:
        json.dump(summary,out,indent=2)
    print(run)
    return 1 if failure else 0


if __name__=='__main__':
    raise SystemExit(main())
