#!/usr/bin/env python3
"""Bounded isolated Isaac IMU clock fixture: rotating cube, no ROS or robot.

Run with Isaac's python.sh and an external archive mount. Retains all samples
across World.reset; does not patch the robot or feed GT to an estimator. A CUDA
preflight in the same container must precede this command.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

from archive_guard import allocate_run, check_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    app = None
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "width": 64, "height": 64,
                             "extra_args": ["--/app/renderer/skipWhileMinimized=false"]})
        import numpy as np
        import omni.graph.core as og
        import usdrt.Sdf
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid
        from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
        from isaacsim.sensors.physics import IMUSensor, _sensor

        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/30)
        world.get_physics_context().set_gravity(0.)
        body = world.scene.add(DynamicCuboid(prim_path="/World/body", name="body", mass=1.,
                                           position=np.array([0., 0., 2.])))
        sensor_path = "/World/body/imu"
        world.scene.add(IMUSensor(prim_path=sensor_path, name="imu", dt=-1.,
                                 translation=np.array([.2, .1, 0.])))
        keys = og.Controller.Keys
        og.Controller.edit({"graph_path": "/ClockProbe", "evaluator_name": "execution"}, {
            keys.CREATE_NODES: [("tick", "omni.graph.action.OnPlaybackTick"),
                                ("reader", "isaacsim.sensors.physics.IsaacReadIMU")],
            keys.CONNECT: [("tick.outputs:tick", "reader.inputs:execIn")],
            keys.SET_VALUES: [("reader.inputs:imuPrim", [usdrt.Sdf.Path(sensor_path)]),
                              ("reader.inputs:useLatestData", False), ("reader.inputs:readGravity", True)]})
        interface = _sensor.acquire_imu_sensor_interface()
        clock = _isaacsim_core_nodes.acquire_interface()
        samples, physics_samples = [], []
        started = time.monotonic()

        def reading(latest):
            value = interface.get_sensor_reading(sensor_path, use_latest_data=latest, read_gravity=True)
            return dict(valid=bool(value.is_valid), time=float(value.time),
                        orientation_xyzw=list(value.orientation),
                        acceleration=[value.lin_acc_x, value.lin_acc_y, value.lin_acc_z],
                        gyro=[value.ang_vel_x, value.ang_vel_y, value.ang_vel_z])

        for epoch in range(2):
            world.reset()
            body.set_angular_velocity(np.array([0., 0., .3]))

            def physics_sample(dt):
                position, quaternion = body.get_world_pose()
                physics_samples.append(dict(epoch=epoch, dt=float(dt), world_time=float(world.current_time),
                    monotonic_time=float(clock.get_sim_time_monotonic()), latest=reading(True),
                    body_position=position.tolist(), body_orientation_wxyz=quaternion.tolist(),
                    body_angular_velocity=body.get_angular_velocity().tolist(),
                    body_linear_velocity=body.get_linear_velocity().tolist()))

            world.add_physics_callback("imu_clock_probe", physics_sample)
            for tick in range(90):
                if time.monotonic()-started > 90:
                    raise TimeoutError("isolated physics clock probe exceeded 90 seconds")
                world.step(render=True)
                position, quaternion = body.get_world_pose()
                samples.append(dict(epoch=epoch, tick=tick, world_time=float(world.current_time),
                    monotonic_time=float(clock.get_sim_time_monotonic()),
                    graph_sensor_time=float(og.Controller.attribute("/ClockProbe/reader.outputs:sensorTime").get()),
                    graph_orientation_xyzw=list(og.Controller.attribute("/ClockProbe/reader.outputs:orientation").get()),
                    body_position=position.tolist(), body_orientation_wxyz=quaternion.tolist(),
                    body_angular_velocity=body.get_angular_velocity().tolist(),
                    body_linear_velocity=body.get_linear_velocity().tolist(),
                    latest=reading(True), periodic=reading(False)))
            world.remove_physics_callback("imu_clock_probe")
            print(f"Completed isolated clock epoch {epoch}", flush=True)
        check_run(output)
        report = dict(completed=True, acceptance=False, physics_dt=1/60, rendering_dt=1/30,
                      no_robot=True, no_ros=True, no_estimator=True, samples=samples, physics_samples=physics_samples,
                      script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        with (output / "clock_probe.json").open("x") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        print(f"CLOCK PROBE COMPLETE {output}", flush=True)
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), type=type(exc).__name__), stream, indent=2)
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
