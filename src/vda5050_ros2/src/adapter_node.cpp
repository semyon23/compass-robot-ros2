#include "vda5050_ros2/adapter_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

#include "vda5050_ros2/factsheet.hpp"
#include "vda5050_ros2/simulated_action.hpp"

using namespace std::chrono_literals;
namespace v = vda5050;

namespace
{
constexpr std::size_t kMaxInstantActionStates = 50;  // старые отбрасываем: clearInstantActions пока нет
constexpr double kTick = 0.1;                        // [с] период tick()
constexpr auto kInitTimeout = 60s;  // initializePosition: на другой карте compass_core перезапускает навигацию

std::string string_param(const v::Action & action, const std::string & key)
{
  for (const auto & p : action.action_parameters.value_or(std::vector<v::ActionParameter>{})) {
    if (p.key == key && p.value.is_string()) {
      return p.value.get<std::string>();
    }
  }
  return "";
}
}

AdapterNode::AdapterNode()
: rclcpp::Node("vda5050_adapter")
{
  const auto host = declare_parameter("broker_host", std::string("127.0.0.1"));
  const auto port = declare_parameter("broker_port", 1883);
  const auto user = declare_parameter("broker_user", std::string(""));
  const auto interface_name = declare_parameter("interface_name", std::string("amr"));
  const auto manufacturer = declare_parameter("manufacturer", std::string("compass"));
  const auto serial_number = declare_parameter("serial_number", std::string("robot-001"));
  const auto visualization_interval = declare_parameter("visualization_interval", 0.5);
  const auto simulated_action = declare_parameter("simulated_action", false);

  // Пароль брокера — только из окружения сервиса, не из конфига.
  const char * password = std::getenv("MQTT_PASS");

  prefix_ = interface_name + "/v3/" + manufacturer + "/" + serial_number + "/";
  actions_ = std::make_unique<ActionRunner>(state_);
  if (simulated_action) {
    add_simulated_action(*actions_);
  }
  factsheet_ = build_factsheet(*this, visualization_interval, actions_->descriptions());

  state_.header.manufacturer = manufacturer;
  state_.header.serial_number = serial_number;
  state_.mobile_robot_position = v::MobileRobotPosition{};  // localized = false, пока нет позы
  state_.maps = std::vector<v::Map>{};
  state_.velocity = v::Velocity{0.0, 0.0, 0.0};
  state_.paused = false;
  state_.new_base_request = false;
  orders_ = std::make_unique<OrderManager>(state_, *actions_);
  drive_ = std::make_unique<Drive>(*this);

  link_ = std::make_unique<MqttLink>(
    host, static_cast<int>(port), "vda5050_" + serial_number, user, password ? password : "",
    std::vector<std::string>{prefix_ + "order", prefix_ + "instantActions"}, prefix_ + "connection");
  publisher_ = std::make_unique<StatePublisher>(*link_, prefix_, manufacturer, serial_number);

  // compass_core публикует режим и статус с transient_local: последнее значение придёт сразу.
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  mode_sub_ = create_subscription<std_msgs::msg::String>(
    "/robot/operating_mode", latched, [this](const std_msgs::msg::String & m) { on_operating_mode(m); });
  status_sub_ = create_subscription<robot_interfaces::msg::RobotStatus>(
    "/compass/status", latched, [this](const robot_interfaces::msg::RobotStatus & m) { on_status(m); });
  // Для Compass (rosbridge): transient_local — UI сразу получает последнее.
  vda_status_pub_ = create_publisher<robot_interfaces::msg::VdaStatus>("/vda5050/status", latched);
  vda_path_pub_ = create_publisher<nav_msgs::msg::Path>("/vda5050/path", latched);
  activate_client_ = create_client<robot_interfaces::srv::ActivateMap>("/robot/maps/activate");
  list_maps_client_ = create_client<robot_interfaces::srv::ListMaps>("/compass/maps/list");
  pose_sub_ = create_subscription<robot_interfaces::msg::RobotPose>(
    "/compass/pose", rclcpp::SensorDataQoS(), [this](const robot_interfaces::msg::RobotPose & m) { on_pose(m); });

  // Таймеры на стенных часах: агенту нужно реальное время, а не /clock симуляции.
  tick_timer_ = rclcpp::create_timer(this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), 100ms,
                                     [this] { tick(); });
  connect_timer_ = rclcpp::create_timer(this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), 3s, [this] {
    if (!link_->connected()) {
      link_->connect(publisher_->connection_json(v::ConnectionState::ConnectionBroken));
    }
  });
  // Библиотеку карт перечитываем раз в 10 с: карту могли импортировать, пересохранить или удалить в Compass.
  maps_timer_ = rclcpp::create_timer(this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), 10s,
                                     [this] { request_maps(); });
  visualization_timer_ = rclcpp::create_timer(
    this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME),
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(visualization_interval)),
    [this] { publisher_->publish_visualization(state_); });

  link_->connect(publisher_->connection_json(v::ConnectionState::ConnectionBroken));
  RCLCPP_INFO(get_logger(), "broker %s:%ld, topics %s*", host.c_str(), port, prefix_.c_str());
}

AdapterNode::~AdapterNode()
{
  // Штатная остановка: fms видит OFFLINE, а не CONNECTION_BROKEN.
  link_->disconnect(publisher_->connection_json(v::ConnectionState::Offline));
}

void AdapterNode::tick()
{
  if (link_->take_link_up()) {
    RCLCPP_INFO(get_logger(), "connected to broker");
    publisher_->publish_connection(v::ConnectionState::Online);
    publisher_->publish_factsheet(factsheet_);
    state_requested_ = true;
  }

  for (const auto & msg : link_->take_messages()) {
    if (msg.topic == prefix_ + "instantActions") {
      handle_instant_actions(msg.payload);
    } else {
      orders_->receive(msg.payload);
    }
  }

  drive_step();
  finish_instant_actions();
  publisher_->update_state(state_, std::exchange(state_requested_, false));
  publish_for_compass();
}

void AdapterNode::publish_for_compass()
{
  robot_interfaces::msg::VdaStatus s;
  s.connection = link_->connected() ? "ONLINE" : "OFFLINE";
  s.order_id = state_.order_id;
  s.order_update_id = state_.order_update_id;
  s.order_active = !v::is_idle(state_);
  s.last_node_id = state_.last_node_id;
  s.driving = state_.driving;
  s.paused = *state_.paused;
  s.new_base_request = *state_.new_base_request;
  for (const auto & e : state_.errors) {
    robot_interfaces::msg::StatusError error;
    error.code = e.error_type;
    error.message = e.error_description.value_or("");
    s.errors.push_back(error);
  }
  const auto now = std::chrono::steady_clock::now();
  if (s != last_vda_status_ || now - vda_status_at_ >= 1s) {
    vda_status_pub_->publish(s);
    last_vda_status_ = s;
    vda_status_at_ = now;
  }

  // Base от узла, где робот, до конца — UI рисует его на карте. Stamp нулевой: сравниваем пути целиком.
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const auto route = orders_->base_route();
  if (route.size() > 1) {
    for (const auto & node : route) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = node.node_position->x;
      p.pose.position.y = node.node_position->y;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
  }
  if (path != last_vda_path_) {
    vda_path_pub_->publish(path);
    last_vda_path_ = path;
  }
}

void AdapterNode::drive_step()
{
  const auto & pos = *state_.mobile_robot_position;

  // Узел пройден, когда робот в его допуске (allowedDeviationXY), — не дожидаясь конца цели Nav2.
  auto near = [&pos](const v::Node & node) {
    const auto & p = *node.node_position;
    const double allowed = p.allowed_deviation_xy && p.allowed_deviation_xy->a > 0 ? p.allowed_deviation_xy->a : 0.5;
    return pos.localized && std::hypot(pos.x - p.x, pos.y - p.y) <= allowed;
  };
  // Промежуточные узлы отрезка проходим на ходу. Последний узел отрезка — остановка: он пройден,
  // когда Nav2 довёз до него, и его SOFT/HARD action'ы стартуют уже стоя.
  for (auto segment = orders_->segment(); segment.size() > 1 && near(segment.front());
       segment = orders_->segment())
  {
    orders_->node_passed(segment.front());
  }
  auto segment = orders_->segment();
  const auto result = drive_->take_result();
  if (result == Drive::Result::Succeeded && !segment.empty() && near(segment.back())) {
    for (const auto & node : segment) {
      orders_->node_passed(node);  // и промежуточные, если робот срезал их на развороте
    }
    segment = orders_->segment();
  } else if (result != Drive::Result::None && !segment.empty()) {
    // compass_core не довёз (препятствие не ушло за его попытки, ~минута) или робот встал не у узла.
    // Робот стоит и ждёт fms: cancelOrder или новый order.
    add_error(v::error_type::kNodeUnreachable, v::ErrorLevel::Critical,
              "cannot reach node " + segment.front().node_id, "nodeId", segment.front().node_id);
  }

  actions_->update(kTick, !state_.driving, *state_.paused);

  const bool blocked = std::any_of(state_.errors.begin(), state_.errors.end(), [](const v::Error & e) {
    return e.error_type == v::error_type::kNodeUnreachable;
  });
  const bool can_drive = (state_.operating_mode == v::OperatingMode::Automatic ||
                          state_.operating_mode == v::OperatingMode::Semiautomatic) &&
    pos.localized && !*state_.paused && !blocked && !actions_->blocks_driving() &&
    state_.safety_state.active_emergency_stop == v::EmergencyStop::None;

  if (!can_drive) {
    if (drive_->active()) {
      drive_->stop();
    }
    return;
  }
  if (segment.empty()) {
    return;
  }
  // Нет цели (старт, после паузы или action'ов) или отрезок продлили — путь до его нового конца.
  // Новая цель заменяет старую на ходу, робот не останавливается.
  if (!drive_->active() || segment.back().sequence_id != goal_end_seq_) {
    std::vector<Drive::Point> points;
    for (const auto & node : segment) {
      points.push_back({node.node_position->x, node.node_position->y});
    }
    if (drive_->follow(pos.x, pos.y, points)) {
      goal_end_seq_ = segment.back().sequence_id;
    }
  }
}

void AdapterNode::finish_instant_actions()
{
  // cancelOrder завершён, когда доработали неотменяемые action'ы order'а.
  if (!actions_->busy()) {
    for (const auto & id : std::exchange(pending_cancels_, {})) {
      set_instant_status(id, v::ActionStatus::Finished);
    }
  }

  // enableMap завершён, когда compass_core перезапустил навигацию с новой картой:
  // теперь можно initializePosition.
  if (enable_ && active_map_id_ == enable_->map_id && compass_ready_) {
    set_instant_status(enable_->action_id, v::ActionStatus::Finished);
    enable_.reset();
  } else if (enable_ && std::chrono::steady_clock::now() > enable_->deadline) {
    add_error(v::error_type::kInvalidInstantAction, v::ErrorLevel::Warning,
              "enableMap: navigation did not start with the map", "actionId", enable_->action_id);
    set_instant_status(enable_->action_id, v::ActionStatus::Failed);
    enable_.reset();
  }

  if (!init_) {
    return;
  }
  const auto & pos = *state_.mobile_robot_position;
  // initializePosition завершён, когда AMCL применил позу: поза сдвинулась после запроса (или робот
  // уже стоял там). AMCL уточняет её по лидару, поэтому точного совпадения не ждём — допуск 0.25.
  const bool applied = std::hypot(pos.x - init_->start_x, pos.y - init_->start_y) > 0.005 ||
    std::hypot(init_->x - init_->start_x, init_->y - init_->start_y) < 0.05;
  if (pos.localized && pos.map_id == init_->map_id && applied &&
      std::hypot(pos.x - init_->x, pos.y - init_->y) < 0.25 &&
      std::fabs(std::remainder(pos.theta - init_->theta, 2 * M_PI)) < 0.25)
  {
    state_.last_node_id = init_->last_node_id;
    state_.last_node_sequence_id = 0;
    set_instant_status(init_->action_id, v::ActionStatus::Finished);
    init_.reset();
  } else if (std::chrono::steady_clock::now() > init_->deadline) {
    add_error(v::error_type::kInvalidInstantAction, v::ErrorLevel::Warning,
              "initializePosition: localization did not take the pose", "actionId", init_->action_id);
    set_instant_status(init_->action_id, v::ActionStatus::Failed);
    init_.reset();
  }
}

void AdapterNode::set_instant_status(const std::string & action_id, v::ActionStatus status)
{
  for (auto & s : state_.instant_action_states) {
    if (s.action_id == action_id) {
      s.action_status = status;
    }
  }
}

void AdapterNode::add_error(const std::string & type, v::ErrorLevel level, const std::string & description,
                            const std::string & reference_key, const std::string & reference_value)
{
  // Одна ошибка каждого типа: новая заменяет старую. Снимает их следующий принятый order (OrderManager).
  std::erase_if(state_.errors, [&type](const v::Error & e) { return e.error_type == type; });
  v::Error error;
  error.error_type = type;
  error.error_level = level;
  error.error_description = description;
  if (!reference_key.empty()) {
    error.error_references = std::vector<v::ErrorReference>{{reference_key, reference_value}};
  }
  state_.errors.push_back(std::move(error));
  RCLCPP_WARN(get_logger(), "%s: %s", type.c_str(), description.c_str());
}

void AdapterNode::handle_instant_actions(const std::string & payload)
{
  v::InstantActions msg;
  try {
    msg = nlohmann::json::parse(payload).get<v::InstantActions>();
  } catch (const std::exception & e) {
    return add_error(v::error_type::kValidationFailure, v::ErrorLevel::Warning,
                     std::string("bad instantActions: ") + e.what());
  }

  auto & states = state_.instant_action_states;
  for (const auto & action : msg.actions) {
    // Повтор того же actionId (fms не дождался state) второй раз не выполняем.
    if (std::any_of(states.begin(), states.end(), [&](const v::ActionState & s) {
        return s.action_id == action.action_id;
      }))
    {
      continue;
    }
    v::ActionState as;
    as.action_id = action.action_id;
    as.action_type = action.action_type;
    as.action_status = run_instant_action(action);
    states.push_back(std::move(as));
    if (states.size() > kMaxInstantActionStates) {
      states.erase(states.begin());
    }
  }
}

v::ActionStatus AdapterNode::run_instant_action(const v::Action & action)
{
  const auto & type = action.action_type;
  if (type == v::action_type::kStateRequest) {
    state_requested_ = true;
  } else if (type == v::action_type::kFactsheetRequest) {
    publisher_->publish_factsheet(factsheet_);
  } else if (type == v::action_type::kStartPause || type == v::action_type::kStopPause) {
    state_.paused = type == v::action_type::kStartPause;  // остановку и продолжение делает drive_step()
  } else if (type == v::action_type::kCancelOrder) {
    // Необязательный параметр orderId: отменить только этот order.
    std::string order_id = string_param(action, "orderId");
    if (order_id.empty()) {
      order_id = state_.order_id;
    }
    if (order_id != state_.order_id || !orders_->cancel()) {
      add_error(v::error_type::kNoOrderToCancel, v::ErrorLevel::Warning, "no active order " + order_id,
                "actionId", action.action_id);
      return v::ActionStatus::Failed;
    }
    drive_->stop();
    // FINISHED — когда доработают неотменяемые action'ы order'а (finish_instant_actions).
    pending_cancels_.push_back(action.action_id);
    return v::ActionStatus::Running;
  } else if (type == v::action_type::kInitializePosition) {
    return initialize_position(action);
  } else if (type == v::action_type::kEnableMap) {
    return enable_map(action);
  } else {
    add_error(v::error_type::kInvalidInstantAction, v::ErrorLevel::Warning, "unsupported action " + type,
              "actionId", action.action_id);
    return v::ActionStatus::Failed;
  }
  return v::ActionStatus::Finished;
}

v::ActionStatus AdapterNode::initialize_position(const v::Action & action)
{
  auto invalid = [&](const std::string & why) {
    add_error(v::error_type::kInvalidInstantAction, v::ErrorLevel::Warning, "initializePosition: " + why,
              "actionId", action.action_id);
    return v::ActionStatus::Failed;
  };

  PendingInit init;
  init.action_id = action.action_id;
  int found = 0;
  for (const auto & p : action.action_parameters.value_or(std::vector<v::ActionParameter>{})) {
    if ((p.key == "x" || p.key == "y" || p.key == "theta") && p.value.is_number()) {
      (p.key == "x" ? init.x : p.key == "y" ? init.y : init.theta) = p.value.get<double>();
      ++found;
    } else if ((p.key == "mapId" || p.key == "lastNodeId") && p.value.is_string()) {
      (p.key == "mapId" ? init.map_id : init.last_node_id) = p.value.get<std::string>();
      ++found;
    }
  }
  if (found != 5) {
    return invalid("x, y, theta, mapId and lastNodeId are required");
  }
  if (!v::is_idle(state_)) {
    return invalid("robot has an active order");
  }
  // Любая карта библиотеки: другую compass_core включит и перезапустит навигацию с этой позой.
  const auto & maps = *state_.maps;
  if (std::none_of(maps.begin(), maps.end(), [&](const v::Map & m) { return m.map_id == init.map_id; })) {
    return invalid("map " + init.map_id + " is not on the robot");
  }
  if (!activate_client_->service_is_ready()) {
    return invalid("compass_core is not available");
  }

  // Поза — в AMCL через compass_core (та же карта — без перезапуска навигации).
  auto request = std::make_shared<robot_interfaces::srv::ActivateMap::Request>();
  request->map_id = init.map_id;
  request->has_initial_pose = true;
  request->initial_pose.x = init.x;
  request->initial_pose.y = init.y;
  request->initial_pose.theta = init.theta;
  activate_client_->async_send_request(
    request, [this, id = action.action_id](rclcpp::Client<robot_interfaces::srv::ActivateMap>::SharedFuture future) {
    const auto response = future.get();
    if (!response->success && init_ && init_->action_id == id) {
      add_error(v::error_type::kInvalidInstantAction, v::ErrorLevel::Warning,
                "initializePosition: " + response->message, "actionId", id);
      set_instant_status(id, v::ActionStatus::Failed);
      init_.reset();
    }
  });
  init.start_x = state_.mobile_robot_position->x;
  init.start_y = state_.mobile_robot_position->y;
  init.deadline = std::chrono::steady_clock::now() + kInitTimeout;
  init_ = init;
  return v::ActionStatus::Running;  // FINISHED — когда AMCL примет позу (finish_instant_actions)
}

v::ActionStatus AdapterNode::enable_map(const v::Action & action)
{
  auto invalid = [&](const std::string & why) {
    add_error(v::error_type::kInvalidInstantAction, v::ErrorLevel::Warning, "enableMap: " + why,
              "actionId", action.action_id);
    return v::ActionStatus::Failed;
  };
  const std::string map_id = string_param(action, "mapId");
  const std::string version = string_param(action, "mapVersion");
  const auto & maps = *state_.maps;
  const auto map = std::find_if(maps.begin(), maps.end(), [&](const v::Map & m) { return m.map_id == map_id; });
  if (map == maps.end()) {
    return invalid("map " + map_id + " is not on the robot");
  }
  if (map->map_version != version) {
    return invalid("map " + map_id + " has version " + map->map_version + ", not " + version);
  }
  if (map->map_status == v::EnableStatus::Enabled) {
    return v::ActionStatus::Finished;
  }
  if (!v::is_idle(state_)) {
    return invalid("robot has an active order");
  }
  if (!activate_client_->service_is_ready()) {
    return invalid("compass_core is not available");
  }
  // compass_core перезапускает навигацию с этой картой. Позы нет — робот не локализован,
  // пока fms не поставит его на граф (initializePosition).
  auto request = std::make_shared<robot_interfaces::srv::ActivateMap::Request>();
  request->map_id = map_id;
  activate_client_->async_send_request(
    request, [this, id = action.action_id](rclcpp::Client<robot_interfaces::srv::ActivateMap>::SharedFuture future) {
      const auto response = future.get();
      if (!response->success && enable_ && enable_->action_id == id) {
        add_error(v::error_type::kInvalidInstantAction, v::ErrorLevel::Warning, "enableMap: " + response->message,
                  "actionId", id);
        set_instant_status(id, v::ActionStatus::Failed);
        enable_.reset();
      }
    });
  // FINISHED — когда навигация с новой картой поднялась (finish_instant_actions).
  enable_ = PendingEnable{action.action_id, map_id, std::chrono::steady_clock::now() + kInitTimeout};
  compass_ready_ = false;  // ready придёт в статусе уже после перезапуска стека
  return v::ActionStatus::Running;
}

void AdapterNode::request_maps()
{
  if (!list_maps_client_->service_is_ready()) {
    return;
  }
  list_maps_client_->async_send_request(
    std::make_shared<robot_interfaces::srv::ListMaps::Request>(),
    [this](rclcpp::Client<robot_interfaces::srv::ListMaps>::SharedFuture future) {
      const auto response = future.get();
      if (!response->success) {
        return;
      }
      library_.clear();
      for (const auto & m : response->maps) {
        // mapVersion — время последнего изменения: пересохранённая карта — новая версия.
        library_.push_back({m.id, std::to_string(m.updated_at.sec), v::EnableStatus::Disabled, m.name});
      }
      update_maps();
    });
}

void AdapterNode::update_maps()
{
  // Все карты библиотеки Compass; включена одна — активная.
  state_.maps = library_;
  for (auto & m : *state_.maps) {
    m.map_status = m.map_id == active_map_id_ ? v::EnableStatus::Enabled : v::EnableStatus::Disabled;
  }
}

void AdapterNode::on_operating_mode(const std_msgs::msg::String & msg)
{
  try {
    // Значения в топике — те же строки, что в VDA ("AUTOMATIC", ...).
    state_.operating_mode = nlohmann::json(msg.data).get<v::OperatingMode>();
  } catch (const std::exception &) {
    RCLCPP_WARN(get_logger(), "unknown operating mode '%s'", msg.data.c_str());
  }
}

void AdapterNode::on_status(const robot_interfaces::msg::RobotStatus & msg)
{
  localization_status_ = msg.localization.status;
  compass_ready_ = msg.state == "ready";
  auto & pos = *state_.mobile_robot_position;
  pos.map_id = msg.map.id;
  pos.deviation_range = msg.localization.covariance;  // σ положения, м
  update_localized();

  // В TEACH_IN map — карта, которую строят: включённой карты нет.
  const std::string active = msg.operating_mode == "TEACH_IN" ? "" : msg.map.id;
  if (active != active_map_id_) {
    active_map_id_ = active;
    request_maps();  // активную карту могли только что сохранить или импортировать
    update_maps();
  }

  auto & power = state_.power_supply;
  power.state_of_charge = msg.battery.present ? std::clamp<double>(msg.battery.percent, 0.0, 100.0) : 0.0;
  if (msg.battery.present) {
    power.battery_voltage = msg.battery.voltage;
  } else {
    power.battery_voltage.reset();
  }
  power.charging = msg.battery.charging;

  // СТОП Compass снимается удалённо (из Compass) — REMOTE.
  state_.safety_state.active_emergency_stop = msg.estop ? v::EmergencyStop::Remote : v::EmergencyStop::None;
}

void AdapterNode::on_pose(const robot_interfaces::msg::RobotPose & msg)
{
  pose_frame_ = msg.frame;
  auto & pos = *state_.mobile_robot_position;
  pos.x = msg.x;
  pos.y = msg.y;
  pos.theta = msg.theta;
  update_localized();

  state_.velocity = v::Velocity{msg.linear, 0.0, msg.angular};  // TODO: vy меканума из одометрии
  state_.driving = std::fabs(msg.linear) > 0.01 || std::fabs(msg.angular) > 0.01;
}

void AdapterNode::update_localized()
{
  // Поза в карте и AMCL уверен. uncertain — ещё localized: робот на линии, σ видна в deviationRange.
  auto & pos = *state_.mobile_robot_position;
  pos.localized = pose_frame_ == "map" && !pos.map_id.empty() &&
    (localization_status_ == "ok" || localization_status_ == "uncertain");
  if (!pos.localized) {
    pos.deviation_range.reset();
  }
}
