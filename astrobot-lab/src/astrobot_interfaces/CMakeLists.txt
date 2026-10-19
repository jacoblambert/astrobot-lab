cmake_minimum_required(VERSION 3.8)
project(astrobot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/LocalizationStatus.msg" "srv/ResetMap.srv"
  "msg/SlamFramePose.msg" "msg/SlamRevision.msg" "srv/GetSlamSnapshot.srv"
  "msg/SlamObservation.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
