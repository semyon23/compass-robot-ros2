#include "vda5050_ros2/factsheet.hpp"

namespace v = vda5050;

namespace
{
v::MobileRobotAction instant_action(const std::string & type, const std::string & description)
{
  v::MobileRobotAction a;
  a.action_type = type;
  a.action_description = description;
  a.action_scopes = {v::ActionScope::Instant};
  a.blocking_types = std::vector{v::BlockingType::None};
  return a;
}
}  // namespace

vda5050::Factsheet build_factsheet(rclcpp::Node & node, double visualization_interval,
                                   const std::vector<vda5050::MobileRobotAction> & order_actions)
{
  auto param = [&node](const std::string & name, auto default_value) {
    return node.declare_parameter("factsheet." + name, default_value);
  };

  v::Factsheet f;

  auto & type = f.type_specification;
  type.series_name = param("series_name", std::string("CompassBot"));
  type.mobile_robot_kinematics = param("kinematics", std::string("DIFFERENTIAL"));  // DIFFERENTIAL | OMNIDIRECTIONAL
  type.mobile_robot_class = param("robot_class", std::string("CARRIER"));
  type.maximum_load_mass = param("maximum_load_mass", 0.0);
  type.localization_types = {v::localization_type::kNatural};          // лидар по карте (AMCL)
  type.navigation_types = {v::navigation_type::kVirtualLineGuided};    // строго по рёбрам графа
  type.supported_zones = std::vector<v::ZoneType>{};                   // зоны не поддерживаем

  // Скорости — как у Nav2 на роботе: по ним fms считает время пути.
  auto & phys = f.physical_parameters;
  phys.minimum_speed = param("minimum_speed", 0.05);
  phys.maximum_speed = param("maximum_speed", 0.5);
  phys.maximum_angular_speed = param("maximum_angular_speed", 1.0);
  phys.maximum_acceleration = param("maximum_acceleration", 0.5);
  phys.maximum_deceleration = param("maximum_deceleration", 0.5);
  phys.length = param("length", 0.5);
  phys.width = param("width", 0.4);
  phys.minimum_height = param("minimum_height", 0.3);
  phys.maximum_height = param("maximum_height", 0.3);

  auto & limits = f.protocol_limits;
  limits.maximum_string_lengths.maximum_id_length = 64;
  limits.maximum_string_lengths.id_numerical_only = false;
  limits.timing.minimum_order_interval = 0.1;   // order'а разбираются каждый тик ноды (10 Гц)
  limits.timing.minimum_state_interval = 0.1;
  limits.timing.default_state_interval = 30.0;  // как в StatePublisher
  limits.timing.visualization_interval = static_cast<float>(visualization_interval);

  auto & features = f.protocol_features;
  features.optional_parameters = {
    {"order.nodes.nodePosition", v::Support::Required, std::string("node coordinates are needed to drive")},
  };
  // Обязательные для любого робота (6.2.3): cancelOrder, startPause, stopPause.
  features.mobile_robot_actions.push_back(instant_action(v::action_type::kCancelOrder, "Cancel the active order"));
  features.mobile_robot_actions.push_back(instant_action(v::action_type::kStartPause, "Pause order execution"));
  features.mobile_robot_actions.push_back(instant_action(v::action_type::kStopPause, "Resume order execution"));
  features.mobile_robot_actions.push_back(instant_action(v::action_type::kStateRequest, "Publish state"));
  features.mobile_robot_actions.push_back(instant_action(v::action_type::kFactsheetRequest, "Publish factsheet"));

  auto init = instant_action(v::action_type::kInitializePosition, "Set robot pose and lastNodeId");
  init.action_parameters = std::vector<v::FactsheetActionParameter>{
    {"x", v::ValueDataType::Number, std::nullopt, false},
    {"y", v::ValueDataType::Number, std::nullopt, false},
    {"theta", v::ValueDataType::Number, std::nullopt, false},
    {"mapId", v::ValueDataType::String, std::nullopt, false},
    {"lastNodeId", v::ValueDataType::String, std::nullopt, false}};
  features.mobile_robot_actions.push_back(init);

  auto enable_map = instant_action(v::action_type::kEnableMap, "Switch to a map of the robot (Compass map library)");
  enable_map.action_parameters = std::vector<v::FactsheetActionParameter>{
    {"mapId", v::ValueDataType::String, std::nullopt, false},
    {"mapVersion", v::ValueDataType::String, std::nullopt, false}};
  features.mobile_robot_actions.push_back(enable_map);

  // Action'ы узлов и рёбер — те, что умеет выполнять ActionRunner.
  features.mobile_robot_actions.insert(features.mobile_robot_actions.end(), order_actions.begin(),
                                       order_actions.end());

  // Контур — прямоугольник по габаритам, центр в начале координат робота.
  const double hx = phys.length / 2.0;
  const double hy = phys.width / 2.0;
  v::Envelope2d contour;
  contour.envelope2d_id = "mechanical";
  contour.vertices = {{hx, hy}, {hx, -hy}, {-hx, -hy}, {-hx, hy}};
  f.mobile_robot_geometry.envelopes2d = std::vector{contour};

  f.load_specification.load_positions = std::vector<std::string>{};  // грузозахвата пока нет

  v::BatteryCharging charging;
  charging.critical_low_charging_level = param("battery_critical_low", 10.0);
  charging.minimum_desired_charging_level = param("battery_minimum_desired", 30.0);
  charging.maximum_desired_charging_level = param("battery_maximum_desired", 90.0);
  v::MobileRobotConfiguration config;
  config.battery_charging = charging;
  f.mobile_robot_configuration = config;
  return f;
}
