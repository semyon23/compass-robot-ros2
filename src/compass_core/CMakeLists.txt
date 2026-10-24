cmake_minimum_required(VERSION 3.8)
project(compass_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(task_orchestrator_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(compass_core
  src/main.cpp
  src/compass_core.cpp
  src/stacks.cpp
  src/navigation.cpp
  src/log_store.cpp
  src/config_store.cpp
  src/settings.cpp
  src/control_lease.cpp
  src/child_process.cpp
  src/map_library.cpp
)
target_include_directories(compass_core PRIVATE include)
ament_target_dependencies(compass_core
  rclcpp rclcpp_action std_msgs std_srvs geometry_msgs nav_msgs nav2_msgs rcl_interfaces sensor_msgs diagnostic_msgs tf2 tf2_ros slam_toolbox task_orchestrator_msgs robot_interfaces
)
target_link_libraries(compass_core yaml-cpp::yaml-cpp nlohmann_json::nlohmann_json)

install(TARGETS compass_core DESTINATION lib/${PROJECT_NAME})

ament_package()
