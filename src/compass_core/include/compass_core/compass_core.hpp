#pragma once

#include <deque>
#include <functional>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <slam_toolbox/srv/pause.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_interfaces/msg/battery.hpp>
#include <robot_interfaces/msg/config_changed.hpp>
#include <robot_interfaces/msg/control_heartbeat.hpp>
#include <robot_interfaces/msg/event.hpp>
#include <robot_interfaces/msg/map_info.hpp>
#include <robot_interfaces/msg/nav_status.hpp>
#include <robot_interfaces/msg/robot_pose.hpp>
#include <robot_interfaces/msg/robot_status.hpp>
#include <robot_interfaces/msg/scan_points.hpp>
#include <robot_interfaces/msg/teleop_command.hpp>
#include <robot_interfaces/srv/acquire_control.hpp>
#include <robot_interfaces/srv/activate_map.hpp>
#include <robot_interfaces/srv/delete_map.hpp>
#include <robot_interfaces/srv/get_events.hpp>
#include <robot_interfaces/srv/get_info.hpp>
#include <robot_interfaces/srv/get_logs.hpp>
#include <robot_interfaces/srv/import_map.hpp>
#include <robot_interfaces/srv/json_call.hpp>
#include <robot_interfaces/srv/list_maps.hpp>
#include <robot_interfaces/srv/nav_start.hpp>
#include <robot_interfaces/srv/release_control.hpp>
#include <robot_interfaces/srv/rename_map.hpp>
#include <robot_interfaces/srv/save_map.hpp>
#include <robot_interfaces/srv/set_operating_mode.hpp>

#include "compass_core/child_process.hpp"
#include "compass_core/config_store.hpp"
#include "compass_core/control_lease.hpp"
#include "compass_core/log_store.hpp"
#include "compass_core/map_library.hpp"

namespace compass_core {

// Сервисная часть робота для Compass. Контракт — compass-ui/docs/ROS_INTERFACES.md.
//   compass_core.cpp — режим работы, статус, аренда управления, джойстик, поза и скан;
//   stacks.cpp       — стеки дочерними процессами (SLAM, навигация), картографирование, карты;
//   navigation.cpp   — локализация, сервисные цели (action'ы Nav2);
//   settings.cpp     — настройки робота (robot.yaml) и их применение, пороги батареи.
class CompassCore : public rclcpp::Node {
public:
  CompassCore();

private:
  using SetOperatingMode = robot_interfaces::srv::SetOperatingMode;
  using AcquireControl = robot_interfaces::srv::AcquireControl;
  using ReleaseControl = robot_interfaces::srv::ReleaseControl;
  using GetInfo = robot_interfaces::srv::GetInfo;
  using GetLogs = robot_interfaces::srv::GetLogs;
  using GetEvents = robot_interfaces::srv::GetEvents;
  using Log = rcl_interfaces::msg::Log;
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;
  using SaveMap = robot_interfaces::srv::SaveMap;
  using ListMaps = robot_interfaces::srv::ListMaps;
  using RenameMap = robot_interfaces::srv::RenameMap;
  using DeleteMap = robot_interfaces::srv::DeleteMap;
  using ImportMap = robot_interfaces::srv::ImportMap;
  using ActivateMap = robot_interfaces::srv::ActivateMap;
  using NavStart = robot_interfaces::srv::NavStart;
  using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;
  using SlamPause = slam_toolbox::srv::Pause;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using JsonCall = robot_interfaces::srv::JsonCall;
  using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;
  using ManageNodes = nav2_msgs::srv::ManageLifecycleNodes;
  using RequestId = std::shared_ptr<rmw_request_id_t>;
  using Details = std::vector<std::pair<std::string, std::string>>;

  // ---------- compass_core.cpp ----------

  // Режим работы
  void set_mode(const std::string & mode);
  void on_set_mode(SetOperatingMode::Request::SharedPtr req, SetOperatingMode::Response::SharedPtr res);

  // Статус и события
  void publish_status();
  void emit_event(const std::string & level, const std::string & code, const std::string & message,
                  const Details & details = {});
  void on_get_info(GetInfo::Request::SharedPtr req, GetInfo::Response::SharedPtr res);
  void load_events();  // последние события из {data_dir}/events.jsonl — лента переживает перезапуск
  void on_get_events(GetEvents::Request::SharedPtr req, GetEvents::Response::SharedPtr res);

  // Журнал: WARN+ из /rosout в памяти и в {data_dir}/logs — UI видит, что было, пока Compass
  // не был открыт, в том числе до перезапуска compass_core.
  void on_rosout(const Log & msg);
  void on_get_logs(GetLogs::Request::SharedPtr req, GetLogs::Response::SharedPtr res);

  // Аренда управления и джойстик
  void on_acquire(AcquireControl::Request::SharedPtr req, AcquireControl::Response::SharedPtr res);
  void on_release(ReleaseControl::Request::SharedPtr req, ReleaseControl::Response::SharedPtr res);
  void on_teleop(const robot_interfaces::msg::TeleopCommand & msg);
  void on_stop(Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void on_stop_release(Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void publish_stop_lock();
  void check_lease();
  void publish_cmd(double linear, double lateral, double angular);  // Twist или TwistStamped
  void publish_zero();

  // Поза и скан
  bool lookup(const std::string & child, std::string & frame,
              geometry_msgs::msg::TransformStamped & tf);
  void publish_pose();
  void on_scan(const sensor_msgs::msg::LaserScan & msg);

  // ---------- stacks.cpp ----------

  // Какой стек нужен сейчас: slam (TEACH_IN), nav (есть активная карта) или никакой.
  std::string wanted_stack() const;
  void check_stack();  // раз в 200 мс: остановить ненужный, запустить нужный, заметить готовность
  void start_stack(const std::string & name);
  void on_stack_exit();
  void stack_ready();
  void check_nav_ready();
  void bring_up_navigation();
  void update_switching();

  // Картографирование (TEACH_IN)
  void on_map(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void on_save(RequestId id, SaveMap::Request::SharedPtr req);
  void on_set_paused(RequestId id, SetBool::Request::SharedPtr req);

  // Библиотека карт и активная карта
  void on_list(ListMaps::Request::SharedPtr req, ListMaps::Response::SharedPtr res);
  void on_rename(RenameMap::Request::SharedPtr req, RenameMap::Response::SharedPtr res);
  void on_delete(DeleteMap::Request::SharedPtr req, DeleteMap::Response::SharedPtr res);
  void on_import(ImportMap::Request::SharedPtr req, ImportMap::Response::SharedPtr res);
  void on_activate(ActivateMap::Request::SharedPtr req, ActivateMap::Response::SharedPtr res);
  void set_active_map(const robot_interfaces::msg::MapRef & map);
  robot_interfaces::msg::MapInfo map_info(const MapEntry & e) const;

  // ---------- navigation.cpp ----------

  // Локализация
  void on_amcl_pose(const PoseCov & msg);
  void publish_initial_pose(const robot_interfaces::msg::Pose2D & pose);
  bool localized() const { return localization_.status == "ok"; }

  // Сервисные цели (action'ы Nav2)
  void on_nav_start(RequestId id, NavStart::Request::SharedPtr req);
  void send_goal(const NavStart::Request & req);
  template <typename Action>
  void send_nav(const std::shared_ptr<rclcpp_action::Client<Action>> & client, const typename Action::Goal & goal);
  void finish_goal(rclcpp_action::ResultCode code, uint16_t error, const std::string & message,
                   const std::string & missed);  // missed — пропущенные точки follow_waypoints, «2, 5»
  void on_nav_cancel(Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void cancel_goal();
  void reply_nav_start(bool success, const std::string & code, const std::string & message);
  void check_nav();  // раз в 500 мс: прогресс цели в /compass/nav_status, ждущая цель, таймаут NavStart
  void publish_nav_status();

  // ---------- settings.cpp ----------

  void on_config(const std::string & op, JsonCall::Request::SharedPtr req, JsonCall::Response::SharedPtr res);
  void apply_config(const nlohmann::json & changed);  // ключи, которые изменились (при старте — все)
  void push_nav_limits();      // motion.* → velocity_smoother Nav2
  void check_battery();        // пороги warn / critical
  double setting(const char * key) const { return config_->values()[key].get<double>(); }

  // ---------- Параметры ----------
  std::string robot_name_, serial_number_, manufacturer_, model_;
  std::string map_frame_, odom_frame_, base_frame_;
  double max_linear_, max_angular_;
  bool cmd_vel_stamped_;  // база ждёт TwistStamped (ros2_control в Jazzy) — так же и twist_mux
  bool holonomic_;        // ездит боком: джойстик и лимит Nav2 по y
  bool use_sim_time_;
  std::string nav_params_, slam_params_;       // nav2.yaml, slam_toolbox.yaml робота — для launch стеков

  // ---------- Состояние ----------
  std::string mode_ = "STARTUP";
  ControlLease lease_;
  robot_interfaces::msg::Battery battery_;
  double linear_ = 0.0, angular_ = 0.0;  // фактические скорости из одометрии
  ControlLease::Clock::time_point last_scan_{};
  ControlLease::Clock::time_point last_teleop_{};
  bool teleop_moving_ = false;  // последняя команда джойстика была ненулевой
  std::unique_ptr<LogStore> logs_;
  std::deque<robot_interfaces::msg::Event> events_;  // последние события, новые в конце
  std::filesystem::path events_file_;
  bool stopped_ = false;  // СТОП из Compass: twist_mux заблокирован, задачи не принимаются
  std::unique_ptr<ConfigStore> config_;
  bool battery_low_ = false, battery_critical_ = false;

  // Стеки: одновременно работает один — SLAM или навигация.
  std::string state_ = "ready";      // ready | switching
  std::string pending_mode_;         // режим, в который переходим со сменой стека (TEACH_IN, SERVICE)
  robot_interfaces::msg::Transition transition_;
  ChildProcess stack_;
  std::string stack_name_;           // slam | nav | пусто
  std::string stack_state_;          // starting | running | stopping | failed
  ChildProcess::Clock::time_point stack_since_{};
  bool restart_stack_ = false;       // перезапустить nav с другой картой
  bool nav_failed_ = false;          // навигация упала — сами не перезапускаем до новой активации
  // Стек nav — один процесс, две части: локализация (map_server + AMCL) поднимается сразу,
  // навигация (planner, controller, …) — после начальной позы: без TF map→odom её костмапы
  // не активируются, и lifecycle_manager бросает запуск.
  bool localization_up_ = false;
  bool navigation_up_ = false;
  bool navigation_request_ = false;  // ждём ответа manage_nodes
  ChildProcess::Clock::time_point nav_query_at_{};  // когда последний раз спрашивали / просили
  std::optional<robot_interfaces::msg::Pose2D> pending_pose_;  // передать AMCL после запуска
  std::optional<robot_interfaces::msg::Pose2D> slam_pose_;     // последняя поза робота в карте SLAM

  // Картографирование и карты
  std::unique_ptr<MapLibrary> maps_;
  std::optional<MapEntry> continue_from_;        // TEACH_IN: достроить эту карту
  std::optional<robot_interfaces::msg::Pose2D> continue_pose_;  // …начиная с этой позы (иначе — с начала карты)
  robot_interfaces::msg::MapRef map_ref_;        // карта, которую строим; id пустой — новая
  robot_interfaces::msg::MapRef active_map_;     // карта для локализации и навигации
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;  // последняя карта от SLAM
  std::size_t map_hash_ = 0, saved_hash_ = 0;    // для unsaved_changes
  bool paused_ = false;

  // Локализация и цели
  robot_interfaces::msg::Localization localization_;
  robot_interfaces::msg::NavStatus nav_;          // текущая или последняя цель Compass
  robot_interfaces::msg::NavStatus prev_nav_;     // вернуть, если новую цель отклонили
  RequestId nav_start_id_;                        // NavStart ждёт, пока Nav2 примет цель
  ChildProcess::Clock::time_point nav_start_deadline_{};
  NavStart::Request::SharedPtr nav_next_;         // новая цель ждёт, пока Nav2 завершит прежнюю
  ChildProcess::Clock::time_point nav_next_at_{}; // …и ещё kNextGoalDelay после этого
  std::function<void()> nav_cancel_;              // отменить цель в Nav2; пусто — Nav2 её ещё не принял
  bool nav_cancel_wanted_ = false;                // отмену попросили до того, как Nav2 принял цель
  bool nav_dirty_ = false;                        // nav_ изменился — опубликовать

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<robot_interfaces::msg::Event>::SharedPtr event_pub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<robot_interfaces::msg::ScanPoints>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;                // или
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;  // по cmd_vel_stamped_
  rclcpp::Publisher<robot_interfaces::msg::NavStatus>::SharedPtr nav_status_pub_;
  rclcpp::Publisher<PoseCov>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<robot_interfaces::msg::ConfigChanged>::SharedPtr config_changed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_lock_pub_;

  rclcpp::Subscription<robot_interfaces::msg::TeleopCommand>::SharedPtr teleop_sub_;
  rclcpp::Subscription<robot_interfaces::msg::ControlHeartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<PoseCov>::SharedPtr amcl_sub_;
  rclcpp::Subscription<Log>::SharedPtr rosout_sub_;

  rclcpp::Client<SerializePoseGraph>::SharedPtr serialize_client_;
  rclcpp::Client<SlamPause>::SharedPtr pause_client_;
  rclcpp::Client<Trigger>::SharedPtr localization_active_client_;
  rclcpp::Client<ManageNodes>::SharedPtr navigation_manage_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr to_pose_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoints_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr through_poses_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> smoother_params_;

  rclcpp::Service<SetOperatingMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<AcquireControl>::SharedPtr acquire_srv_;
  rclcpp::Service<ReleaseControl>::SharedPtr release_srv_;
  rclcpp::Service<GetInfo>::SharedPtr info_srv_;
  rclcpp::Service<GetLogs>::SharedPtr logs_srv_;
  rclcpp::Service<GetEvents>::SharedPtr events_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_release_srv_;
  rclcpp::Service<Trigger>::SharedPtr ping_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<SaveMap>::SharedPtr save_srv_;
  rclcpp::Service<SetBool>::SharedPtr pause_srv_;
  rclcpp::Service<ListMaps>::SharedPtr list_srv_;
  rclcpp::Service<RenameMap>::SharedPtr rename_srv_;
  rclcpp::Service<DeleteMap>::SharedPtr delete_srv_;
  rclcpp::Service<ImportMap>::SharedPtr import_srv_;
  rclcpp::Service<ActivateMap>::SharedPtr activate_srv_;
  rclcpp::Service<NavStart>::SharedPtr nav_start_srv_;
  rclcpp::Service<Trigger>::SharedPtr nav_cancel_srv_;
  std::vector<rclcpp::Service<JsonCall>::SharedPtr> config_srvs_;

  rclcpp::TimerBase::SharedPtr status_timer_, pose_timer_, lease_timer_, stack_timer_, nav_timer_;
};

// Общие мелочи для всех файлов compass_core.
double yaw_of(const geometry_msgs::msg::Quaternion & q);
bool clean_name(std::string & name);  // имя карты от человека: обрезать, 1..64 символа

template <typename ResPtr>
void fail(const ResPtr & res, const char * code, const std::string & message)
{
  res->success = false;
  res->error_code = code;
  res->message = message;
}

}  // namespace compass_core
