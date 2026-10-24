#pragma once

#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_interfaces/msg/battery.hpp>
#include <robot_interfaces/msg/control_heartbeat.hpp>
#include <robot_interfaces/msg/event.hpp>
#include <robot_interfaces/msg/robot_pose.hpp>
#include <robot_interfaces/msg/robot_status.hpp>
#include <robot_interfaces/msg/scan_points.hpp>
#include <robot_interfaces/msg/teleop_command.hpp>
#include <robot_interfaces/srv/acquire_control.hpp>
#include <robot_interfaces/srv/get_info.hpp>
#include <robot_interfaces/srv/release_control.hpp>
#include <robot_interfaces/srv/set_operating_mode.hpp>

#include "compass_core/control_lease.hpp"

namespace compass_core {

// Сервисная часть робота для Compass, срез 1: режим работы, статус, аренда управления,
// джойстик, поза и скан. Контракт — compass-ui/docs/ROS_INTERFACES.md.
class CompassCore : public rclcpp::Node {
public:
  CompassCore();

private:
  using SetOperatingMode = robot_interfaces::srv::SetOperatingMode;
  using AcquireControl = robot_interfaces::srv::AcquireControl;
  using ReleaseControl = robot_interfaces::srv::ReleaseControl;
  using GetInfo = robot_interfaces::srv::GetInfo;
  using Trigger = std_srvs::srv::Trigger;
  using Details = std::vector<std::pair<std::string, std::string>>;

  // Режим работы
  void set_mode(const std::string & mode);
  void on_set_mode(SetOperatingMode::Request::SharedPtr req, SetOperatingMode::Response::SharedPtr res);

  // Статус и события
  void publish_status();
  void emit_event(const std::string & level, const std::string & code, const std::string & message,
                  const Details & details = {});
  void on_get_info(GetInfo::Request::SharedPtr req, GetInfo::Response::SharedPtr res);

  // Аренда управления и джойстик
  void on_acquire(AcquireControl::Request::SharedPtr req, AcquireControl::Response::SharedPtr res);
  void on_release(ReleaseControl::Request::SharedPtr req, ReleaseControl::Response::SharedPtr res);
  void on_teleop(const robot_interfaces::msg::TeleopCommand & msg);
  void on_stop(Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void check_lease();
  void publish_zero();

  // Поза и скан
  bool lookup(const std::string & child, std::string & frame,
              geometry_msgs::msg::TransformStamped & tf);
  void publish_pose();
  void on_scan(const sensor_msgs::msg::LaserScan & msg);

  // Параметры
  std::string robot_name_, serial_number_, manufacturer_, model_;
  std::string map_frame_, odom_frame_, base_frame_;
  double max_linear_, max_angular_;

  // Состояние
  std::string mode_ = "STARTUP";
  ControlLease lease_;
  robot_interfaces::msg::Battery battery_;
  double linear_ = 0.0, angular_ = 0.0;  // фактические скорости из одометрии
  ControlLease::Clock::time_point last_scan_{};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<robot_interfaces::msg::Event>::SharedPtr event_pub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<robot_interfaces::msg::ScanPoints>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Subscription<robot_interfaces::msg::TeleopCommand>::SharedPtr teleop_sub_;
  rclcpp::Subscription<robot_interfaces::msg::ControlHeartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  rclcpp::Service<SetOperatingMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<AcquireControl>::SharedPtr acquire_srv_;
  rclcpp::Service<ReleaseControl>::SharedPtr release_srv_;
  rclcpp::Service<GetInfo>::SharedPtr info_srv_;
  rclcpp::Service<Trigger>::SharedPtr ping_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;

  rclcpp::TimerBase::SharedPtr status_timer_, pose_timer_, lease_timer_;
};

}  // namespace compass_core
