#include "compass_core/compass_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <unordered_set>

#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.hpp>

using namespace std::chrono_literals;

namespace compass_core {

namespace {

// Режимы, в которых оператор может ездить джойстиком (ARCHITECTURE.md §3).
bool teleop_allowed(const std::string & mode)
{
  return mode == "MANUAL" || mode == "INTERVENED" || mode == "SERVICE" || mode == "TEACH_IN";
}

constexpr auto kLeaseTimeout = 2s;     // нет heartbeat и джойстика — аренда снимается
constexpr auto kTeleopTimeout = 500ms; // джойстик замолчал — останавливаем робота
constexpr auto kScanPeriod = 200ms;    // скан для UI — 5 Гц
constexpr std::size_t kScanPoints = 360;
constexpr double kCameraCell = 0.025;   // [м] точки камеры для UI — по одной на клетку
constexpr std::size_t kLogsKept = 1000;  // WARN+ из /rosout в памяти
constexpr int kLogDays = 7;              // и столько дней — в файлах
constexpr std::size_t kEventsKept = 200; // лента событий для UI

}  // namespace

// Угол поворота вокруг z из кватерниона — робот на плоскости, крен и тангаж не нужны.
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Имя карты от человека: без пробелов по краям, от 1 до 64 символов (UTF-8).
bool clean_name(std::string & name)
{
  const auto first = name.find_first_not_of(" \t\n");
  if (first == std::string::npos) {
    return false;
  }
  name = name.substr(first, name.find_last_not_of(" \t\n") - first + 1);
  const auto chars = std::count_if(name.begin(), name.end(), [](char c) { return (c & 0xC0) != 0x80; });
  return chars <= 64;
}

CompassCore::CompassCore() : Node("compass_core")
{
  robot_name_ = declare_parameter("robot_name", "compass-robot");
  serial_number_ = declare_parameter("serial_number", "");
  manufacturer_ = declare_parameter("manufacturer", "");
  model_ = declare_parameter("model", "");
  map_frame_ = declare_parameter("map_frame", "map");
  odom_frame_ = declare_parameter("odom_frame", "odom");
  base_frame_ = declare_parameter("base_frame", "base_link");
  max_linear_ = declare_parameter("max_linear", 0.5);
  max_angular_ = declare_parameter("max_angular", 1.0);
  cmd_vel_stamped_ = declare_parameter("cmd_vel_stamped", false);
  holonomic_ = declare_parameter("holonomic", false);
  robot_radius_ = declare_parameter("robot_radius", 0.3);
  // Не меньше threshold_to_consider у GoalCritic / PathFollowCritic MPPI (nav2.yaml): 1.4 м по умолчанию.
  route_loop_gap_ = declare_parameter("route_loop_gap", 1.5);
  use_sim_time_ = get_parameter("use_sim_time").as_bool();  // передаём в launch стеков
  // Параметры стеков, которые compass_core запускает сам, — из каталога конфигов робота.
  nav_params_ = declare_parameter("nav_params", "");
  slam_params_ = declare_parameter("slam_params", "");
  const std::string data_dir = declare_parameter("data_dir", "/var/lib/robot");
  maps_ = std::make_unique<MapLibrary>(data_dir);
  logs_ = std::make_unique<LogStore>(std::filesystem::path(data_dir) / "logs", kLogsKept, kLogDays);
  events_file_ = std::filesystem::path(data_dir) / "events.jsonl";
  load_events();
  // Настройки: robot.yaml в {data_dir}/config. Пока его нет — из схемы и из параметров выше.
  const auto config_dir = std::filesystem::path(data_dir) / "config";
  config_ = std::make_unique<ConfigStore>(
    config_dir, declare_parameter("config_schema", ""),
    nlohmann::json{{"identity.name", robot_name_}, {"identity.serial_number", serial_number_},
                   {"identity.manufacturer", manufacturer_}, {"identity.model", model_},
                   {"motion.max_linear", max_linear_}, {"motion.max_angular", max_angular_}});
  apply_config(nlohmann::json(nullptr));  // null — все ключи

  // Активная карта переживает перезапуск; навигация с ней поднимется, когда робот выйдет из STARTUP.
  if (const auto e = maps_->get(maps_->active_id())) {
    active_map_.id = e->id;
    active_map_.name = e->name;
  }
  localization_.status = "none";

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Латченые: новый подписчик (агент VDA, вкладка Compass) сразу получает текущее значение.
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  mode_pub_ = create_publisher<std_msgs::msg::String>("/robot/operating_mode", latched);
  status_pub_ = create_publisher<robot_interfaces::msg::RobotStatus>("/compass/status", latched);
  nav_status_pub_ = create_publisher<robot_interfaces::msg::NavStatus>("/compass/nav_status", latched);
  event_pub_ = create_publisher<robot_interfaces::msg::Event>("/compass/events", rclcpp::QoS(50).reliable());
  pose_pub_ = create_publisher<robot_interfaces::msg::RobotPose>("/compass/pose", rclcpp::SensorDataQoS().keep_last(1));
  scan_pub_ = create_publisher<robot_interfaces::msg::ScanPoints>("/compass/scan_points", rclcpp::SensorDataQoS().keep_last(1));
  camera_pub_ = create_publisher<robot_interfaces::msg::ScanPoints>("/compass/camera_points", rclcpp::SensorDataQoS().keep_last(1));
  // Вход teleop в twist_mux — того же типа, что ждёт twist_mux (use_stamped).
  if (cmd_vel_stamped_) {
    cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_teleop", rclcpp::QoS(1));
  } else {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", rclcpp::QoS(1));
  }
  initial_pose_pub_ = create_publisher<PoseCov>("/initialpose", rclcpp::QoS(1).reliable());
  plan_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 1);  // QoS — как у planner_server
  config_changed_pub_ = create_publisher<robot_interfaces::msg::ConfigChanged>("/compass/config/changed", latched);
  // Блокировка twist_mux (locks.stop в twist_mux.yaml): true — ни джойстик, ни Nav2 не едут.
  stop_lock_pub_ = create_publisher<std_msgs::msg::Bool>("compass_stop_lock", latched);

  teleop_sub_ = create_subscription<robot_interfaces::msg::TeleopCommand>(
    "/compass/teleop", rclcpp::QoS(1),
    [this](const robot_interfaces::msg::TeleopCommand & msg) { on_teleop(msg); });
  heartbeat_sub_ = create_subscription<robot_interfaces::msg::ControlHeartbeat>(
    "/compass/control/heartbeat", rclcpp::QoS(1),
    [this](const robot_interfaces::msg::ControlHeartbeat & msg) {
      lease_.touch(msg.lease_id, ControlLease::Clock::now());
    });
  // Датчики — best_effort: такой подписчик совместим и с reliable, и с best_effort издателем.
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    declare_parameter("scan_topic", "scan"), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan & msg) { on_scan(msg); });
  // Камера глубины — необязательна (пустой топик — нет камеры): её препятствия ниже луча лидара
  // показываем в UI, как скан.
  const auto camera_topic = declare_parameter("camera_topic", std::string(""));
  camera_min_height_ = declare_parameter("camera_min_height", 0.03);
  camera_max_height_ = declare_parameter("camera_max_height", 0.15);
  camera_max_range_ = declare_parameter("camera_max_range", 1.5);
  if (!camera_topic.empty()) {
    camera_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      camera_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::PointCloud2 & msg) { on_camera(msg); });
  }
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    declare_parameter("odom_topic", "odom"), rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry & msg) {
      linear_ = msg.twist.twist.linear.x;
      angular_ = msg.twist.twist.angular.z;
    });
  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
    declare_parameter("battery_topic", "battery_state"), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::BatteryState & msg) {
      battery_.present = true;
      battery_.percent = std::isnan(msg.percentage) ? 0.0f : msg.percentage * 100.0f;  // в BatteryState 0..1
      battery_.voltage = msg.voltage;
      battery_.charging =
        msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      check_battery();
    });
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", latched, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { on_map(msg); });
  amcl_sub_ = create_subscription<PoseCov>(
    "/amcl_pose", latched, [this](const PoseCov & msg) { on_amcl_pose(msg); });
  // volatile — совместима с любым издателем; что было до запуска compass_core, не нужно.
  rosout_sub_ = create_subscription<Log>(
    "/rosout", rclcpp::QoS(100).reliable(), [this](const Log & msg) { on_rosout(msg); });

  serialize_client_ = create_client<SerializePoseGraph>("/slam_toolbox/serialize_map");
  pause_client_ = create_client<SlamPause>("/slam_toolbox/pause_new_measurements");
  localization_active_client_ = create_client<Trigger>("/lifecycle_manager_localization/is_active");
  navigation_manage_client_ = create_client<ManageNodes>("/lifecycle_manager_navigation/manage_nodes");
  to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
  waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");
  smoother_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "velocity_smoother");

  using namespace std::placeholders;
  set_mode_srv_ = create_service<SetOperatingMode>(
    "/robot/set_operating_mode", std::bind(&CompassCore::on_set_mode, this, _1, _2));
  acquire_srv_ = create_service<AcquireControl>(
    "/compass/control/acquire", std::bind(&CompassCore::on_acquire, this, _1, _2));
  release_srv_ = create_service<ReleaseControl>(
    "/compass/control/release", std::bind(&CompassCore::on_release, this, _1, _2));
  info_srv_ = create_service<GetInfo>("/compass/get_info", std::bind(&CompassCore::on_get_info, this, _1, _2));
  logs_srv_ = create_service<GetLogs>("/compass/logs/history", std::bind(&CompassCore::on_get_logs, this, _1, _2));
  ping_srv_ = create_service<Trigger>(
    "/compass/ping", [](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { res->success = true; });
  stop_srv_ = create_service<Trigger>("/compass/stop", std::bind(&CompassCore::on_stop, this, _1, _2));
  stop_release_srv_ = create_service<Trigger>(
    "/compass/stop/release", std::bind(&CompassCore::on_stop_release, this, _1, _2));
  events_srv_ = create_service<GetEvents>(
    "/compass/events/history", std::bind(&CompassCore::on_get_events, this, _1, _2));
  // save, set_paused и nav/start отвечают позже — когда ответят slam_toolbox или Nav2:
  // обработчик не ждёт, ответ уходит из колбэка клиента (send_response).
  save_srv_ = create_service<SaveMap>(
    "/compass/mapping/save", [this](RequestId id, SaveMap::Request::SharedPtr req) { on_save(id, req); });
  pause_srv_ = create_service<SetBool>(
    "/compass/mapping/set_paused", [this](RequestId id, SetBool::Request::SharedPtr req) { on_set_paused(id, req); });
  list_srv_ = create_service<ListMaps>("/compass/maps/list", std::bind(&CompassCore::on_list, this, _1, _2));
  rename_srv_ = create_service<RenameMap>("/compass/maps/rename", std::bind(&CompassCore::on_rename, this, _1, _2));
  delete_srv_ = create_service<DeleteMap>("/compass/maps/delete", std::bind(&CompassCore::on_delete, this, _1, _2));
  import_srv_ = create_service<ImportMap>("/compass/maps/import", std::bind(&CompassCore::on_import, this, _1, _2));
  activate_srv_ = create_service<ActivateMap>("/robot/maps/activate", std::bind(&CompassCore::on_activate, this, _1, _2));
  nav_start_srv_ = create_service<NavStart>(
    "/compass/nav/start", [this](RequestId id, NavStart::Request::SharedPtr req) { on_nav_start(id, req); });
  nav_cancel_srv_ = create_service<Trigger>("/compass/nav/cancel", std::bind(&CompassCore::on_nav_cancel, this, _1, _2));
  for (const std::string op : {"get_schema", "get", "apply", "history", "rollback"}) {
    config_srvs_.push_back(create_service<JsonCall>(
      "/compass/config/" + op,
      [this, op](JsonCall::Request::SharedPtr req, JsonCall::Response::SharedPtr res) { on_config(op, req, res); }));
  }

  status_timer_ = create_wall_timer(1s, [this] { publish_status(); });
  pose_timer_ = create_wall_timer(100ms, [this] { publish_pose(); });
  lease_timer_ = create_wall_timer(100ms, [this] { check_lease(); });
  stack_timer_ = create_wall_timer(200ms, [this] { check_stack(); });
  nav_timer_ = create_wall_timer(500ms, [this] { check_nav(); });

  std_msgs::msg::String mode;
  mode.data = mode_;
  mode_pub_->publish(mode);
  robot_interfaces::msg::ConfigChanged changed;
  changed.version = config_->version();
  config_changed_pub_->publish(changed);
  publish_status();
  RCLCPP_INFO(get_logger(), "Режим работы: %s — жду TF %s → %s", mode_.c_str(), odom_frame_.c_str(),
              base_frame_.c_str());
}

// ---------- Режим работы ----------

void CompassCore::set_mode(const std::string & mode)
{
  if (mode == mode_) {
    return;
  }
  cancel_goal();  // цель Compass — только в SERVICE, задание fms — только в AUTOMATIC
  mode_ = mode;
  std_msgs::msg::String msg;
  msg.data = mode_;
  mode_pub_->publish(msg);

  // В режимах без джойстика аренда не нужна: снимаем и останавливаем ручное движение.
  if (!teleop_allowed(mode_) && lease_.active()) {
    lease_.clear();
    publish_zero();
  }
  RCLCPP_INFO(get_logger(), "Режим работы: %s", mode_.c_str());
  emit_event("info", "MODE_CHANGED", "Operating mode: " + mode_, {{"mode", mode_}});
  publish_status();
}

void CompassCore::on_set_mode(SetOperatingMode::Request::SharedPtr req, SetOperatingMode::Response::SharedPtr res)
{
  const std::string & m = req->mode;
  if (m != "AUTOMATIC" && m != "INTERVENED" && m != "MANUAL" && m != "SERVICE" && m != "TEACH_IN") {
    return fail(res, "INVALID_INPUT", "неизвестный режим: " + m);
  }
  if (mode_ == "STARTUP") {
    return fail(res, "MODE_NOT_ALLOWED", "робот ещё запускается");
  }
  if (state_ != "ready") {
    return fail(res, "BUSY", "робот переключает режим или запускает навигацию");
  }
  res->success = true;
  if (m == mode_) {
    return;
  }
  // SERVICE ⇄ TEACH_IN — со сменой стека: ответ сразу, дальше ведёт check_stack(),
  // ход перехода — в status.transition.
  if (mode_ == "TEACH_IN") {
    if (m != "SERVICE") {
      return fail(res, "MODE_NOT_ALLOWED", "сначала завершите картографирование");
    }
    pending_mode_ = "SERVICE";
    update_switching();
    return publish_status();
  }
  if (m == "TEACH_IN") {
    if (mode_ != "SERVICE") {
      return fail(res, "MODE_NOT_ALLOWED", "картографирование включается из режима SERVICE");
    }
    std::optional<MapEntry> base;
    if (!req->continue_from.empty()) {
      base = maps_->get(req->continue_from);
      if (!base) {
        return fail(res, "NOT_FOUND", "карта не найдена");
      }
      if (!base->has_posegraph) {
        return fail(res, "INVALID_INPUT", "у карты «" + base->name + "» нет posegraph — продолжить нельзя");
      }
    }
    cancel_goal();
    continue_from_ = base;
    // Достраиваем активную карту, а робот на ней локализован — SLAM стартует с его текущей позы.
    // Иначе slam_toolbox считает, что робот стоит там, где эту карту начинали.
    continue_pose_.reset();
    geometry_msgs::msg::TransformStamped tf;
    std::string frame;
    if (base && base->id == active_map_.id && localized() && lookup(base_frame_, frame, tf) && frame == map_frame_) {
      continue_pose_.emplace();
      continue_pose_->x = tf.transform.translation.x;
      continue_pose_->y = tf.transform.translation.y;
      continue_pose_->theta = yaw_of(tf.transform.rotation);
    }
    map_ref_ = robot_interfaces::msg::MapRef{};
    if (base) {
      map_ref_.id = base->id;
      map_ref_.name = base->name;
    }
    pending_mode_ = "TEACH_IN";
    update_switching();
    return publish_status();
  }
  // «Передать fms»: командует агент VDA (условия — ROS_INTERFACES.md, «Режим работы»).
  // «Перехватить»: INTERVENED, если у агента есть order, иначе MANUAL — выбирает UI.
  if (m == "AUTOMATIC") {
    if (!navigation_up_ || !localized()) {
      return fail(res, "MODE_NOT_ALLOWED", "нужны запущенная навигация и локализация на карте");
    }
    if (stopped_) {
      return fail(res, "MODE_NOT_ALLOWED", "снимите СТОП");
    }
    if (nav_.state == "active") {
      return fail(res, "MODE_NOT_ALLOWED", "робот выполняет сервисную цель — отмените её");
    }
  } else if (m == "INTERVENED" && mode_ != "AUTOMATIC") {
    // «Перехватить» посреди order'а: агент стоит, order fms сохраняется до «Передать fms».
    return fail(res, "MODE_NOT_ALLOWED", "вмешательство — только из режима работы с fms");
  }
  set_mode(m);
}

// ---------- Статус и события ----------

void CompassCore::publish_status()
{
  robot_interfaces::msg::RobotStatus s;
  s.stamp = now();
  s.operating_mode = mode_;
  s.state = state_;
  s.transition = transition_;
  const bool mapping = mode_ == "TEACH_IN" || pending_mode_ == "TEACH_IN";
  s.map = mapping ? map_ref_ : active_map_;
  s.localization = localization_;
  s.battery = battery_;
  s.control.holder_id = lease_.holder_id();
  s.control.holder_name = lease_.holder_name();
  s.estop = stopped_;  // программный СТОП из Compass; аппаратного сигнала пока нет
  s.mapping.active = mode_ == "TEACH_IN";
  s.mapping.paused = paused_;
  s.mapping.unsaved_changes = s.mapping.active && map_hash_ != saved_hash_;
  auto add_stack = [&s](const std::string & name, const std::string & state) {
    robot_interfaces::msg::StackState st;
    st.name = name;
    st.state = state;
    s.stacks.push_back(st);
  };
  if (stack_name_ == "slam") {
    add_stack("slam", stack_state_);
  } else if (stack_name_ == "nav") {
    // Навигация ждёт позу — «starting», пока процесс жив.
    add_stack("localization", stack_state_);
    add_stack("navigation", stack_state_ == "running" && !navigation_up_ ? "starting" : stack_state_);
  }
  status_pub_->publish(s);
  publish_stop_lock();  // и раз в секунду: twist_mux мог перезапуститься и не знать о СТОП
}

void CompassCore::emit_event(const std::string & level, const std::string & code,
                             const std::string & message, const Details & details)
{
  robot_interfaces::msg::Event e;
  // Время по часам робота, а не sim time: события читает человек, лента хранится между запусками.
  e.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  e.id = std::to_string(rclcpp::Time(e.stamp).nanoseconds());  // уникален и после перезапуска
  e.level = level;
  e.code = code;
  e.message = message;
  for (const auto & [key, value] : details) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    e.details.push_back(kv);
  }
  event_pub_->publish(e);

  events_.push_back(e);
  if (events_.size() > kEventsKept) {
    events_.pop_front();
  }
  nlohmann::json kv_json = nlohmann::json::object();
  for (const auto & kv : e.details) {
    kv_json[kv.key] = kv.value;
  }
  std::ofstream(events_file_, std::ios::app)
    << nlohmann::json{{"id", e.id}, {"sec", e.stamp.sec}, {"nanosec", e.stamp.nanosec}, {"level", level},
                      {"code", code}, {"message", message}, {"details", kv_json}}.dump()
    << "\n";
}

// Лента событий с диска. Файл растёт, поэтому при старте оставляем в нём только последние kEventsKept.
void CompassCore::load_events()
{
  std::ifstream in(events_file_);
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  if (lines.size() > kEventsKept) {
    lines.erase(lines.begin(), lines.end() - kEventsKept);
    std::ofstream out(events_file_);
    for (const auto & line : lines) {
      out << line << "\n";
    }
  }
  for (const auto & line : lines) {
    const auto j = nlohmann::json::parse(line, nullptr, false);
    if (j.is_discarded()) {
      continue;
    }
    robot_interfaces::msg::Event e;
    e.id = j.value("id", "");
    e.stamp.sec = j.value("sec", 0);
    e.stamp.nanosec = j.value("nanosec", 0u);
    e.level = j.value("level", "");
    e.code = j.value("code", "");
    e.message = j.value("message", "");
    // Сначала в переменную: items() от временного объекта перебирал бы уже освобождённую память.
    const nlohmann::json details = j.value("details", nlohmann::json::object());
    for (const auto & [key, value] : details.items()) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value.is_string() ? value.get<std::string>() : value.dump();
      e.details.push_back(kv);
    }
    events_.push_back(e);
  }
}

void CompassCore::on_get_events(GetEvents::Request::SharedPtr req, GetEvents::Response::SharedPtr res)
{
  for (auto it = events_.rbegin(); it != events_.rend(); ++it) {
    if (req->limit > 0 && res->events.size() >= req->limit) {
      break;
    }
    res->events.push_back(*it);
  }
  res->success = true;
}

void CompassCore::on_get_info(GetInfo::Request::SharedPtr, GetInfo::Response::SharedPtr res)
{
  res->success = true;
  res->robot_name = robot_name_;
  res->serial_number = serial_number_;
  res->manufacturer = manufacturer_;
  res->model = model_;
  res->robot_radius = static_cast<float>(robot_radius_);
  const char * distro = std::getenv("ROS_DISTRO");
  for (const auto & [key, value] : Details{{"compass_core", "0.3.0"}, {"ros", distro ? distro : ""}}) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    res->versions.push_back(kv);
  }
  // UI показывает только то, что робот умеет.
  res->capabilities = {"scan_stream", "mapping", "continue_mapping", "navigation"};
  if (holonomic_) {
    res->capabilities.push_back("holonomic");  // UI показывает джойстик «вбок»
  }
}

// ---------- Журнал ----------

void CompassCore::on_rosout(const Log & msg)
{
  if (msg.level < Log::WARN) {
    return;
  }
  // Время по часам робота: в симуляции ноды ставят sim time, а UI нужно настоящее.
  const int64_t ms = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() / 1'000'000;
  logs_->add({ms, msg.level, msg.name, msg.msg, msg.file, msg.function, msg.line});
}

void CompassCore::on_get_logs(GetLogs::Request::SharedPtr req, GetLogs::Response::SharedPtr res)
{
  const uint8_t min_level = std::max<uint8_t>(req->min_level, Log::WARN);
  for (const auto & e : logs_->entries()) {
    if (e.level < min_level) {
      continue;
    }
    Log l;
    l.stamp.sec = static_cast<int32_t>(e.ms / 1000);
    l.stamp.nanosec = static_cast<uint32_t>(e.ms % 1000) * 1'000'000;
    l.level = e.level;
    l.name = e.name;
    l.msg = e.msg;
    l.file = e.file;
    l.function = e.function;
    l.line = e.line;
    res->entries.push_back(l);
  }
  if (req->limit > 0 && res->entries.size() > req->limit) {
    res->entries.erase(res->entries.begin(), res->entries.end() - req->limit);
  }
  res->success = true;
}

// ---------- Аренда управления и джойстик ----------

void CompassCore::on_acquire(AcquireControl::Request::SharedPtr req, AcquireControl::Response::SharedPtr res)
{
  if (!teleop_allowed(mode_)) {
    return fail(res, "MODE_NOT_ALLOWED", "в режиме " + mode_ + " ездить джойстиком нельзя");
  }
  if (req->client_id.empty()) {
    return fail(res, "INVALID_INPUT", "пустой client_id");
  }
  const std::string previous_name = lease_.holder_name();
  const auto r = lease_.acquire(req->client_id, req->client_name, req->force, ControlLease::Clock::now());
  if (!r.granted) {
    return fail(res, "CONTROL_LOCKED", "управляет " + lease_.holder_name());
  }
  if (!r.previous_holder_id.empty()) {
    publish_zero();  // джойстик прежнего владельца больше не действует
    emit_event("warning", "CONTROL_TAKEN", "Control taken by " + req->client_name,
               {{"previous_holder_id", r.previous_holder_id}, {"previous_holder_name", previous_name}});
  }
  res->success = true;
  res->lease_id = r.lease_id;
  publish_status();
}

void CompassCore::on_release(ReleaseControl::Request::SharedPtr req, ReleaseControl::Response::SharedPtr res)
{
  if (!lease_.release(req->lease_id)) {
    return fail(res, "CONTROL_LOCKED", "это не ваша аренда");
  }
  publish_zero();
  res->success = true;
  publish_status();
}

void CompassCore::on_teleop(const robot_interfaces::msg::TeleopCommand & msg)
{
  if (!lease_.touch(msg.lease_id, ControlLease::Clock::now())) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Джойстик без действующей аренды — игнорирую");
    return;
  }
  const double linear = std::clamp(static_cast<double>(msg.linear), -max_linear_, max_linear_);
  const double lateral = holonomic_ ? std::clamp(static_cast<double>(msg.lateral), -max_linear_, max_linear_) : 0.0;
  const double angular = std::clamp(static_cast<double>(msg.angular), -max_angular_, max_angular_);
  publish_cmd(linear, lateral, angular);
  last_teleop_ = ControlLease::Clock::now();
  teleop_moving_ = linear != 0.0 || lateral != 0.0 || angular != 0.0;
  // Оператор взялся за джойстик — сервисная цель отменяется: twist_mux отдал бы скорость
  // джойстику, но Nav2 продолжал бы считать, что едет к цели.
  if (teleop_moving_) {
    cancel_goal();
  }
}

// СТОП: twist_mux блокирует все входы скорости, цель Compass отменяется, новые не принимаются
// (status.estop), пока оператор не снимет СТОП в Compass.
void CompassCore::on_stop(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  publish_zero();
  cancel_goal();  // и задание fms: агент видит status.estop и новых не шлёт
  if (!stopped_) {
    stopped_ = true;
    emit_event("warning", "STOP_PRESSED", "Stop pressed");
  }
  publish_status();
  res->success = true;
}

void CompassCore::on_stop_release(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  if (stopped_) {
    stopped_ = false;
    emit_event("info", "STOP_RELEASED", "Stop released");
  }
  publish_status();
  res->success = true;
}

void CompassCore::publish_stop_lock()
{
  std_msgs::msg::Bool lock;
  lock.data = stopped_;
  stop_lock_pub_->publish(lock);
}

void CompassCore::check_lease()
{
  const auto now = ControlLease::Clock::now();
  // Джойстик замолчал (отпустили, пропала связь) — останавливаем сами: twist_mux по таймауту
  // просто перестаёт публиковать, а база держит последнюю скорость (так ведёт себя и Gazebo).
  if (teleop_moving_ && now - last_teleop_ > kTeleopTimeout) {
    publish_zero();
  }
  if (lease_.expire(now, kLeaseTimeout)) {
    publish_zero();
    RCLCPP_INFO(get_logger(), "Аренда снята: нет heartbeat %ld с", static_cast<long>(kLeaseTimeout.count()));
    publish_status();
  }
}

void CompassCore::publish_cmd(double linear, double lateral, double angular)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = linear;
  t.linear.y = lateral;
  t.angular.z = angular;
  if (cmd_stamped_pub_) {
    geometry_msgs::msg::TwistStamped ts;
    ts.header.stamp = now();
    ts.header.frame_id = base_frame_;
    ts.twist = t;
    cmd_stamped_pub_->publish(ts);
  } else {
    cmd_pub_->publish(t);
  }
}

void CompassCore::publish_zero()
{
  publish_cmd(0.0, 0.0, 0.0);
  teleop_moving_ = false;
}

// ---------- Поза и скан ----------

// Мировой фрейм для UI: map, если работает локализация или SLAM, иначе odom.
bool CompassCore::lookup(const std::string & child, std::string & frame,
                         geometry_msgs::msg::TransformStamped & tf)
{
  for (const std::string & f : {map_frame_, odom_frame_}) {
    try {
      tf = tf_buffer_->lookupTransform(f, child, tf2::TimePointZero);
      frame = f;
      return true;
    } catch (const tf2::TransformException &) {
    }
  }
  return false;
}

void CompassCore::publish_pose()
{
  std::string frame;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookup(base_frame_, frame, tf)) {
    return;  // нет TF — UI покажет «нет данных о положении»
  }
  if (mode_ == "STARTUP") {
    set_mode("SERVICE");  // базовый стек жив: есть TF робота
  }
  robot_interfaces::msg::RobotPose p;
  if (stack_name_ == "slam" && stack_state_ == "running" && frame == map_frame_) {
    slam_pose_.emplace();
    slam_pose_->x = tf.transform.translation.x;
    slam_pose_->y = tf.transform.translation.y;
    slam_pose_->theta = yaw_of(tf.transform.rotation);
  }
  p.stamp = tf.header.stamp;
  p.frame = frame;
  p.x = tf.transform.translation.x;
  p.y = tf.transform.translation.y;
  p.theta = yaw_of(tf.transform.rotation);
  p.linear = static_cast<float>(linear_);
  p.angular = static_cast<float>(angular_);
  pose_pub_->publish(p);
}

void CompassCore::on_scan(const sensor_msgs::msg::LaserScan & msg)
{
  const auto t = ControlLease::Clock::now();
  if (t - last_scan_ < kScanPeriod) {
    return;
  }
  last_scan_ = t;

  std::string frame;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookup(msg.header.frame_id, frame, tf)) {
    return;
  }
  // Лазер лежит в плоскости пола: хватает сдвига и поворота вокруг z.
  const double tx = tf.transform.translation.x;
  const double ty = tf.transform.translation.y;
  const double yaw = yaw_of(tf.transform.rotation);
  const double c = std::cos(yaw), s = std::sin(yaw);

  robot_interfaces::msg::ScanPoints out;
  out.stamp = msg.header.stamp;
  out.frame = frame;
  const std::size_t step = std::max<std::size_t>(1, msg.ranges.size() / kScanPoints);
  for (std::size_t i = 0; i < msg.ranges.size(); i += step) {
    const float r = msg.ranges[i];
    if (!std::isfinite(r) || r < msg.range_min || r > msg.range_max) {
      continue;
    }
    const double a = msg.angle_min + i * msg.angle_increment;
    const double lx = r * std::cos(a), ly = r * std::sin(a);
    out.points.push_back(static_cast<float>(tx + c * lx - s * ly));
    out.points.push_back(static_cast<float>(ty + s * lx + c * ly));
  }
  scan_pub_->publish(out);
}

// Облако камеры для UI: те же точки, что camera_layer костмапа считает препятствиями, — в полосе высот
// над полом (z мирового фрейма) и не дальше camera_max_range от камеры; по одной на клетку kCameraCell.
void CompassCore::on_camera(const sensor_msgs::msg::PointCloud2 & msg)
{
  const auto t = ControlLease::Clock::now();
  if (t - last_camera_ < kScanPeriod) {
    return;
  }
  last_camera_ = t;

  std::string frame;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookup(msg.header.frame_id, frame, tf)) {
    return;
  }
  const auto & r = tf.transform.rotation;
  const auto & p = tf.transform.translation;
  const tf2::Transform to_world(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(p.x, p.y, p.z));

  robot_interfaces::msg::ScanPoints out;
  out.stamp = msg.header.stamp;
  out.frame = frame;
  std::unordered_set<std::int64_t> cells;
  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*z) || std::hypot(*x, *y, *z) > camera_max_range_) {
      continue;
    }
    const tf2::Vector3 w = to_world * tf2::Vector3(*x, *y, *z);
    if (w.z() < camera_min_height_ || w.z() > camera_max_height_) {
      continue;
    }
    // Номер клетки: ±100 км по каждой оси без совпадений.
    const auto cx = static_cast<std::int64_t>(std::floor(w.x() / kCameraCell));
    const auto cy = static_cast<std::int64_t>(std::floor(w.y() / kCameraCell));
    if (cells.insert(cx * 10'000'000 + cy).second) {
      out.points.push_back(static_cast<float>(w.x()));
      out.points.push_back(static_cast<float>(w.y()));
    }
  }
  camera_pub_->publish(out);
}

}  // namespace compass_core
