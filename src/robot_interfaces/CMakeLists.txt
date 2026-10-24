cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/Battery.msg"
  "msg/ControlState.msg"
  "msg/StatusError.msg"
  "msg/Event.msg"
  "msg/RobotPose.msg"
  "msg/ScanPoints.msg"
  "msg/TeleopCommand.msg"
  "msg/ControlHeartbeat.msg"
  "srv/SetOperatingMode.srv"
  "srv/GetInfo.srv"
  "srv/AcquireControl.srv"
  "srv/ReleaseControl.srv"
  DEPENDENCIES builtin_interfaces diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
