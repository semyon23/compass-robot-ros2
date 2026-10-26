cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

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
  "msg/Pose2D.msg"
  "msg/Transition.msg"
  "msg/MapRef.msg"
  "msg/MappingState.msg"
  "msg/MapInfo.msg"
  "msg/Localization.msg"
  "msg/StackState.msg"
  "msg/NavStatus.msg"
  "msg/ConfigChanged.msg"
  "msg/VdaStatus.msg"
  "srv/SetOperatingMode.srv"
  "srv/GetInfo.srv"
  "srv/AcquireControl.srv"
  "srv/ReleaseControl.srv"
  "srv/SaveMap.srv"
  "srv/ListMaps.srv"
  "srv/RenameMap.srv"
  "srv/DeleteMap.srv"
  "srv/ImportMap.srv"
  "srv/ActivateMap.srv"
  "srv/NavStart.srv"
  "srv/GetLogs.srv"
  "srv/JsonCall.srv"
  "srv/GetEvents.srv"
  DEPENDENCIES builtin_interfaces diagnostic_msgs rcl_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
