# GLIM configuration

The configuration the mission gates were validated with (Oct 2026): CPU odometry on the
selected LiDAR stream (`/astrobot_0/lidar_selected/points`), lunar gravity, global loop
closure disabled, and the extension modules

- `libastrobot_revision_module.so` (astrobot_glim_extensions): publishes `slam_raw/revisions`
  for the terrain node;
- `libastrobot_abs_constraint_module.so` (astrobot_glim_extensions): fuses the gated RTK fixes
  (`/astrobot_0/localization/absolute_pose`) into the odometry smoother;
- `librviz_viewer.so` (glim_ros): TF and visualisation topics.

The localization supervisor copies this directory per epoch and writes the initial pose and the
RTK topic into the copy (`astrobot_localization/src/child_config.cpp`); edit it here, not in the
run copy. Select another directory with `glim_config:=<dir>`.
