cmake_minimum_required(VERSION 3.8)
project(compass_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(compass_core
  src/main.cpp
  src/compass_core.cpp
  src/control_lease.cpp
)
target_include_directories(compass_core PRIVATE include)
ament_target_dependencies(compass_core
  rclcpp std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs diagnostic_msgs tf2 tf2_ros robot_interfaces
)

install(TARGETS compass_core DESTINATION lib/${PROJECT_NAME})

ament_package()
