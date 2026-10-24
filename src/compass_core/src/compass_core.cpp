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

// Угол поворота вокруг z из кватерниона — робот на плоскости, крен и тангаж не нужны.
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

template <typename ResPtr>
void fail(const ResPtr & res, const char * code, const std::string & message)
{
  res->success = false;
  res->error_code = code;
  res->message = message;
}

constexpr auto kLeaseTimeout = 2s;     // нет heartbeat и джойстика — аренда снимается
constexpr auto kScanPeriod = 200ms;    // скан для UI — 5 Гц
constexpr std::size_t kScanPoints = 360;

}  // namespace

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

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Латченые: новый подписчик (агент VDA, вкладка Compass) сразу получает текущее значение.
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  mode_pub_ = create_publisher<std_msgs::msg::String>("/robot/operating_mode", latched);
  status_pub_ = create_publisher<robot_interfaces::msg::RobotStatus>("/compass/status", latched);
  event_pub_ = create_publisher<robot_interfaces::msg::Event>("/compass/events", rclcpp::QoS(50).reliable());
  pose_pub_ = create_publisher<robot_interfaces::msg::RobotPose>("/compass/pose", rclcpp::SensorDataQoS().keep_last(1));
  scan_pub_ = create_publisher<robot_interfaces::msg::ScanPoints>("/compass/scan_points", rclcpp::SensorDataQoS().keep_last(1));
  // Вход teleop в twist_mux; сторожевой таймер — у twist_mux (timeout 0.5 с).
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", rclcpp::QoS(1));

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
    });

  using namespace std::placeholders;
  set_mode_srv_ = create_service<SetOperatingMode>(
    "/robot/set_operating_mode", std::bind(&CompassCore::on_set_mode, this, _1, _2));
  acquire_srv_ = create_service<AcquireControl>(
    "/compass/control/acquire", std::bind(&CompassCore::on_acquire, this, _1, _2));
  release_srv_ = create_service<ReleaseControl>(
    "/compass/control/release", std::bind(&CompassCore::on_release, this, _1, _2));
  info_srv_ = create_service<GetInfo>("/compass/get_info", std::bind(&CompassCore::on_get_info, this, _1, _2));
  ping_srv_ = create_service<Trigger>(
    "/compass/ping", [](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { res->success = true; });
  stop_srv_ = create_service<Trigger>("/compass/stop", std::bind(&CompassCore::on_stop, this, _1, _2));

  status_timer_ = create_wall_timer(1s, [this] { publish_status(); });
  pose_timer_ = create_wall_timer(100ms, [this] { publish_pose(); });
  lease_timer_ = create_wall_timer(200ms, [this] { check_lease(); });

  std_msgs::msg::String mode;
  mode.data = mode_;
  mode_pub_->publish(mode);
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
  // Срез 1: только SERVICE и MANUAL. AUTOMATIC и INTERVENED появятся вместе с локализацией
  // и агентом VDA, TEACH_IN — вместе с картографированием.
  if (m != "SERVICE" && m != "MANUAL") {
    return fail(res, "MODE_NOT_ALLOWED", "режим " + m + " пока не поддерживается");
  }
  set_mode(m);
  res->success = true;
}

// ---------- Статус и события ----------

void CompassCore::publish_status()
{
  robot_interfaces::msg::RobotStatus s;
  s.stamp = now();
  s.operating_mode = mode_;
  s.state = "ready";
  s.battery = battery_;
  s.control.holder_id = lease_.holder_id();
  s.control.holder_name = lease_.holder_name();
  s.estop = false;  // сигнала e-stop пока нет (вопрос в контракте)
  status_pub_->publish(s);
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
  for (const auto & [key, value] : Details{{"compass_core", "0.1.0"}, {"ros", distro ? distro : ""}}) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    res->versions.push_back(kv);
  }
  // UI показывает только то, что робот умеет. Картографирование и навигация — следующие срезы.
  res->capabilities = {"scan_stream"};
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
}

void CompassCore::on_stop(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  // Срез 1: только обнулить джойстик. Блокировка twist_mux и отмена задач — со следующими срезами.
  publish_zero();
  res->success = true;
}

void CompassCore::check_lease()
{
  if (lease_.expire(ControlLease::Clock::now(), kLeaseTimeout)) {
    publish_zero();
    RCLCPP_INFO(get_logger(), "Аренда снята: нет heartbeat %ld с", static_cast<long>(kLeaseTimeout.count()));
    publish_status();
  }
}

void CompassCore::publish_zero()
{
  cmd_pub_->publish(geometry_msgs::msg::Twist{});
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
