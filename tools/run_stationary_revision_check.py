#!/usr/bin/env python3
"""Bounded stationary wiring check: recorder and probes start BEFORE the stack.

Requires an already running simulator with GT map->odom disabled. Starts Phase 3
without a goal sender; never publishes motion. Keeps all attempts on the external
drive. Run the frame audit separately after this process finishes.
"""
import argparse
import json
import os
from pathlib import Path
import select
import shlex
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.serialization import serialize_message
from astrobot_interfaces.srv import GetSlamSnapshot
from archive_guard import ARCHIVE_ROOT, check_run
from localization_smoke import Probe


def stop(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        # Only the dedicated process group started by this script is stopped.
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-root', type=Path, default=Path('/archive') if Path('/archive').is_dir() else ARCHIVE_ROOT)
    parser.add_argument('--wall-seconds', type=float, default=60)
    parser.add_argument('--seed', type=int, default=42, help='Declared simulator seed; must match the already running simulator')
    args = parser.parse_args()
    if not 30 <= args.wall_seconds <= 300:
        parser.error('stationary check must last 30..300 wall seconds')
    tools = Path(__file__).resolve().parent
    recorder = subprocess.Popen([sys.executable, str(tools / 'record_revision_fixture.py'),
        '--archive-root', str(args.archive_root), '--wall-seconds', str(args.wall_seconds + 20)],
        stdout=subprocess.PIPE, text=True, start_new_session=True)
    launch, manifest, node, run = None, None, None, None
    try:
        if not select.select([recorder.stdout], [], [], 10)[0]:
            raise RuntimeError('recorder did not allocate archive promptly')
        run = Path(recorder.stdout.readline().strip())
        if not run.is_absolute():
            raise RuntimeError('recorder did not report a valid run path')
        check_run(run)
        print(run, flush=True)
        rclpy.init()
        node = Probe()
        client = node.create_client(GetSlamSnapshot, '/astrobot_0/slam_raw/get_snapshot')
        command = ['ros2', 'launch', 'astrobot_launch', 'phase3_nav.launch.py', 'slam_start_delay:=0.0']
        env = dict(os.environ, ROS_LOG_DIR=str(run / 'ros-log'))
        with (run / 'stationary_launch.log').open('x') as log:
            launch = subprocess.Popen(command, stdout=log, stderr=log, env=env, start_new_session=True)
            start, future, failure = time.monotonic(), None, None
            while time.monotonic() - start < args.wall_seconds:
                if launch.poll() is not None or recorder.poll() is not None:
                    failure = 'launch or recorder exited early'
                    break
                rclpy.spin_once(node, timeout_sec=.05)
                if future is None and time.monotonic() - start > 15 and client.service_is_ready():
                    future = client.call_async(GetSlamSnapshot.Request())
                    manifest = subprocess.Popen([sys.executable, str(tools / 'write_run_manifest.py'),
                        '--output', str(run / 'runtime_manifest.json'), '--seed', str(args.seed),
                        '--launch-command', shlex.join(command)], stdout=log, stderr=log, start_new_session=True)
                if node.nonzero_commands:
                    failure = 'unexpected motion command in stationary check'
                    break
                check_run(run)
            if manifest is None:
                failure = failure or 'runtime provenance was not captured'
            else:
                try:
                    if manifest.wait(timeout=10) != 0:
                        failure = failure or 'runtime provenance collection failed'
                except subprocess.TimeoutExpired:
                    failure = failure or 'runtime provenance collection timed out'
            summary = node.summary()
            summary['evaluation_mode'] = 'assisted_slam_stationary_engineering_fixture'
            summary['declared_simulator_seed'] = args.seed
            snapshot_summary = {'success': False, 'reason': 'snapshot unavailable during running stack'}
            if future is not None and future.done():
                response = future.result()
                snapshot = response.snapshot
                snapshot_summary = dict(success=response.success, reason=response.reason,
                    revision=snapshot.revision, frames=len(snapshot.frames), session=snapshot.session_id,
                    epoch=snapshot.localization_epoch, full_snapshot=snapshot.full_snapshot)
                with (run / 'snapshot.cdr').open('xb') as out:
                    out.write(serialize_message(snapshot))
            summary['pass'] = bool(summary['pass'] and snapshot_summary['success'] and failure is None)
            summary['orchestration_failure'] = failure
            for name, result in (('stationary_smoke.json', summary), ('snapshot_probe.json', snapshot_summary)):
                with (run / name).open('x') as out:
                    json.dump(result, out, indent=2)
            print(json.dumps({'stationary_pass': summary['pass'], 'counts': summary['counts'],
                              'snapshot': snapshot_summary}), flush=True)
            stop(launch)
        recorder.wait(timeout=40)
        return 0 if summary['pass'] and recorder.returncode == 0 else 1
    except Exception as exc:
        if run is not None:
            with (run / 'orchestration_failure.json').open('x') as out:
                json.dump({'pass': False, 'failure': str(exc)}, out, indent=2)
        raise
    finally:
        stop(launch)
        stop(manifest)
        stop(recorder)
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
