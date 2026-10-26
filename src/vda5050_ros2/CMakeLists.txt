cmake_minimum_required(VERSION 3.8)
project(vda5050_ros2)

set(CMAKE_CXX_STANDARD 20)  # std::erase_if

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_executable(vda5050_adapter
  src/main.cpp
  src/adapter_node.cpp
  src/mqtt_link.cpp
  src/state_publisher.cpp
  src/factsheet.cpp
  src/order_manager.cpp
  src/drive.cpp
  src/action_runner.cpp
  src/simulated_action.cpp
)
# third_party/vda5050 — копия ~/fleet-manager/vda5050 (типы сообщений VDA 5050 v3), подключается как <vda5050.hpp>.
target_include_directories(vda5050_adapter PRIVATE include third_party/vda5050)
ament_target_dependencies(vda5050_adapter rclcpp rclcpp_action std_msgs nav_msgs nav2_msgs robot_interfaces)
target_link_libraries(vda5050_adapter nlohmann_json::nlohmann_json PahoMqttCpp::paho-mqttpp3)

install(TARGETS vda5050_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
