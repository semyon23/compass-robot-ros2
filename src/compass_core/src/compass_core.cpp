#include "compass_core/compass_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

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
constexpr std::size_t kLogsKept = 1000;  // WARN+ из /rosout в памяти
constexpr int kLogDays = 7;              // и столько дней — в файлах

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
  use_sim_time_ = get_parameter("use_sim_time").as_bool();  // передаём в launch стеков
  const std::string data_dir = declare_parameter("data_dir", "/var/lib/robot");
  maps_ = std::make_unique<MapLibrary>(data_dir);
  logs_ = std::make_unique<LogStore>(std::filesystem::path(data_dir) / "logs", kLogsKept, kLogDays);
  // Настройки: robot.yaml в {data_dir}/config. Пока его нет — из схемы и из параметров выше.
  const auto config_dir = std::filesystem::path(data_dir) / "config";
  config_ = std::make_unique<ConfigStore>(
    config_dir, declare_parameter("config_schema", ""),
    nlohmann::json{{"identity.name", robot_name_}, {"identity.serial_number", serial_number_},
                   {"identity.manufacturer", manufacturer_}, {"identity.model", model_},
                   {"motion.max_linear", max_linear_}, {"motion.max_angular", max_angular_}});
  tasks_template_ = declare_parameter("tasks_template", "");
  tasks_config_ = (config_dir / "tasks.yaml").string();
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
  // Вход teleop в twist_mux.
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", rclcpp::QoS(1));
  initial_pose_pub_ = create_publisher<PoseCov>("/initialpose", rclcpp::QoS(1).reliable());
  config_changed_pub_ = create_publisher<robot_interfaces::msg::ConfigChanged>("/compass/config/changed", latched);

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
    "scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan & msg) { on_scan(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry & msg) {
      linear_ = msg.twist.twist.linear.x;
      angular_ = msg.twist.twist.angular.z;
    });
  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::BatteryState & msg) {
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
  task_client_ = rclcpp_action::create_client<ExecuteTask>(this, "/task_orchestrator/execute_task");
  cancel_tasks_client_ = create_client<CancelTasks>("/task_orchestrator/cancel_tasks");
  stop_tasks_client_ = create_client<StopTasks>("/task_orchestrator/stop");
  orchestrator_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "task_orchestrator");
  smoother_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "velocity_smoother");
  reload_tasks_client_ = create_client<ReloadTasks>("/task_orchestrator/reload_config");

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
  // save, set_paused и nav/start отвечают позже — когда ответят slam_toolbox или оркестратор:
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
  if (mode_ == "SERVICE") {
    cancel_goal();  // сервисные цели — только в SERVICE
  }
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
    map_ref_ = robot_interfaces::msg::MapRef{};
    if (base) {
      map_ref_.id = base->id;
      map_ref_.name = base->name;
    }
    pending_mode_ = "TEACH_IN";
    update_switching();
    return publish_status();
  }
  // AUTOMATIC и INTERVENED появятся вместе с агентом VDA.
  if (m != "SERVICE" && m != "MANUAL") {
    return fail(res, "MODE_NOT_ALLOWED", "режим " + m + " пока не поддерживается");
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
  s.estop = false;  // сигнала e-stop пока нет (вопрос в контракте)
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
  update_admission();
  reload_tasks();
}

void CompassCore::emit_event(const std::string & level, const std::string & code,
                             const std::string & message, const Details & details)
{
  robot_interfaces::msg::Event e;
  e.stamp = now();
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
}

void CompassCore::on_get_info(GetInfo::Request::SharedPtr, GetInfo::Response::SharedPtr res)
{
  res->success = true;
  res->robot_name = robot_name_;
  res->serial_number = serial_number_;
  res->manufacturer = manufacturer_;
  res->model = model_;
  const char * distro = std::getenv("ROS_DISTRO");
  for (const auto & [key, value] : Details{{"compass_core", "0.3.0"}, {"ros", distro ? distro : ""}}) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    res->versions.push_back(kv);
  }
  // UI показывает только то, что робот умеет.
  res->capabilities = {"scan_stream", "mapping", "continue_mapping", "navigation"};
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
  geometry_msgs::msg::Twist t;
  t.linear.x = std::clamp(static_cast<double>(msg.linear), -max_linear_, max_linear_);
  t.angular.z = std::clamp(static_cast<double>(msg.angular), -max_angular_, max_angular_);
  cmd_pub_->publish(t);
  last_teleop_ = ControlLease::Clock::now();
  teleop_moving_ = t.linear.x != 0.0 || t.angular.z != 0.0;
  // Оператор взялся за джойстик — сервисная цель отменяется: twist_mux отдал бы скорость
  // джойстику, но Nav2 продолжал бы считать, что едет к цели.
  if (teleop_moving_) {
    cancel_goal();
  }
}

void CompassCore::on_stop(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  publish_zero();
  stop_tasks();  // все задачи с cancel_on_stop — в том числе цели Compass
  res->success = true;
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

void CompassCore::publish_zero()
{
  cmd_pub_->publish(geometry_msgs::msg::Twist{});
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

}  // namespace compass_core
