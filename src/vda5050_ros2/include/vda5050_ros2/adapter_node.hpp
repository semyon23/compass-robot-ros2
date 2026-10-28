#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <robot_interfaces/msg/robot_status.hpp>
#include <robot_interfaces/msg/vda_status.hpp>
#include <robot_interfaces/srv/activate_map.hpp>
#include <robot_interfaces/srv/list_maps.hpp>
#include <std_msgs/msg/string.hpp>
#include <vda5050.hpp>

#include "vda5050_ros2/action_runner.hpp"
#include "vda5050_ros2/drive.hpp"
#include "vda5050_ros2/mqtt_link.hpp"
#include "vda5050_ros2/order_manager.hpp"
#include "vda5050_ros2/state_publisher.hpp"

// Агент VDA 5050 на роботе. Связь с fms — MQTT (MqttLink), состояние робота — из compass_core,
// order'а — OrderManager, езда — Drive (маршрут «по линии» через compass_core).
// Вся логика в одном потоке ROS executor'а: колбэки подписок и таймеров, входящие MQTT —
// через очередь MqttLink, которую разбирает tick().
class AdapterNode : public rclcpp::Node {
public:
  AdapterNode();
  ~AdapterNode() override;

private:
  void tick();
  void drive_step();
  void update_planned_path();  // state.plannedPath из пути compass_core
  void handle_instant_actions(const std::string & payload);
  vda5050::ActionStatus run_instant_action(const vda5050::Action & action);
  vda5050::ActionStatus initialize_position(const vda5050::Action & action);
  vda5050::ActionStatus enable_map(const vda5050::Action & action);
  void request_maps();  // библиотека карт Compass → library_
  void update_maps();   // state.maps из library_ и активной карты
  void finish_instant_actions();  // cancelOrder и initializePosition, которые выполняются не сразу
  void set_instant_status(const std::string & action_id, vda5050::ActionStatus status);
  void add_error(const std::string & type, vda5050::ErrorLevel level, const std::string & description,
                 const std::string & reference_key = "", const std::string & reference_value = "");
  void on_operating_mode(const std_msgs::msg::String & msg);
  void on_status(const robot_interfaces::msg::RobotStatus & msg);
  void on_pose(const robot_interfaces::msg::RobotPose & msg);
  void update_localized();
  void publish_for_compass();  // /vda5050/status и /vda5050/path

  std::string prefix_;  // "amr/v3/<manufacturer>/<serialNumber>/"
  vda5050::Factsheet factsheet_;
  vda5050::State state_;
  std::string pose_frame_;          // map | odom — из /compass/pose
  std::string localization_status_; // ok | uncertain | lost | none — из /compass/status
  bool compass_ready_ = false;      // compass_core не переключает режим и не перезапускает стек
  std::vector<vda5050::Map> library_;  // карты Compass; mapDescriptor — имя карты
  std::string active_map_id_;          // активная карта Compass; пусто — нет (или TEACH_IN)
  bool state_requested_ = false;    // stateRequest или переподключение — отправить state сразу

  std::unique_ptr<MqttLink> link_;
  std::unique_ptr<StatePublisher> publisher_;
  std::unique_ptr<ActionRunner> actions_;
  std::unique_ptr<OrderManager> orders_;
  std::unique_ptr<Drive> drive_;
  std::uint32_t goal_end_seq_ = 0;  // sequenceId узла, до которого отправлен маршрут compass_core

  std::vector<std::string> pending_cancels_;  // cancelOrder ждут неотменяемые action'ы
  struct PendingInit {                        // initializePosition ждёт, когда AMCL примет позу
    std::string action_id;
    double x = 0, y = 0, theta = 0;
    double start_x = 0, start_y = 0;  // поза робота в момент запроса
    std::string map_id;
    std::string last_node_id;
    std::chrono::steady_clock::time_point deadline;
  };
  std::optional<PendingInit> init_;
  struct PendingEnable {                      // enableMap ждёт перезапуска навигации с картой
    std::string action_id;
    std::string map_id;
    std::chrono::steady_clock::time_point deadline;
  };
  std::optional<PendingEnable> enable_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<robot_interfaces::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<robot_interfaces::msg::RobotPose>::SharedPtr pose_sub_;
  rclcpp::Publisher<robot_interfaces::msg::VdaStatus>::SharedPtr vda_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vda_path_pub_;
  robot_interfaces::msg::VdaStatus last_vda_status_;
  std::chrono::steady_clock::time_point vda_status_at_;
  nav_msgs::msg::Path last_vda_path_;
  rclcpp::Client<robot_interfaces::srv::ActivateMap>::SharedPtr activate_client_;
  rclcpp::Client<robot_interfaces::srv::ListMaps>::SharedPtr list_maps_client_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr connect_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::TimerBase::SharedPtr maps_timer_;
};
