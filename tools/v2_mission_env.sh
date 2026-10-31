# Shared v2 mission configuration, sourced by tools/v2_gate_run.sh and tools/v2_demo.sh after
# MODE and SEED are set: dev container, ROS setup, v2 stack arguments, per-mode mission arguments
# (as tools/phase4_mission_gate_run.sh), eval duration and the sim's environment.
DEV=astrobot-selected-dev
SRC="source /etc/ros_setup.sh && source /workspace/astrobot-lab/astrobot-lab/install/setup.bash && export PYTHONPATH=/archive/contact-deps-oct21:/workspace/astrobot-lab/tools:\$PYTHONPATH && cd /workspace/astrobot-lab"
SHARE=/workspace/astrobot-lab/astrobot-lab/install/astrobot_launch/share/astrobot_launch

V2_ARGS="slam_stack:=v2 nav2_params:=$SHARE/config/nav2_v2.yaml exploration_map_topic:=/terrain_shadow/exploration_map reveal_distance_m:=1.5 reachable_seed_distance_m:=1.0 home_approach_radius_m:=0.5 return_behavior_tree:=$SHARE/behavior_trees/navigate_to_pose_lunar_recovery_precise.xml nav_through_poses_bt_xml:=$SHARE/behavior_trees/navigate_through_poses_v2.xml mission_visit_margin_m:=0.15"
if [ "$MODE" = "sample_return" ]; then
  export OMNILRS_RESOURCE_ENABLED=true OMNILRS_RESOURCE_MODE=deterministic OMNILRS_RESOURCE_CENTERS=11.875,12.133,8.0,1.0
  LAUNCH_ARGS="mission_mode:=sample_return sample_success_threshold:=0.75 resource_weight_scale:=25.0 info_weight:=0.3 resource_gradient_weight:=10.0 home_penalty_weight:=0.5 home_radius_m:=19.0 max_goal_distance_m:=5.0 max_duration_sec:=1200.0 exploration_start_delay:=35.0"
  EVAL_DUR=1500
elif [ "$MODE" = "teleop_click" ]; then
  # Demo only: the operator clicks goals in the Ops console; no mission end, no gate.
  export OMNILRS_RESOURCE_ENABLED=true
  LAUNCH_ARGS="mission_mode:=teleop_click exploration_start_delay:=35.0"
  EVAL_DUR=3600
else
  export OMNILRS_RESOURCE_ENABLED=true
  LAUNCH_ARGS="mission_mode:=explore_radius coverage_goal_fraction:=0.70 home_radius_m:=8.0 max_goal_distance_m:=5.0 mission_cell_size_m:=2.0 mission_candidate_limit:=120 max_duration_sec:=1800.0 breadcrumb_return_enabled:=true use_through_poses_return:=false return_breadcrumb_stride_m:=3.0 return_breadcrumb_min_distance_m:=1.0 exploration_start_delay:=35.0"
  EVAL_DUR=2600
fi
export OMNILRS_ENVIRONMENT_SEED="$SEED"
export OMNILRS_CAMERA_ENABLED=false  # gates never render the camera; v2_demo.sh --camera overrides
