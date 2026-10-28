#include "compass_core/compass_core.hpp"

using json = nlohmann::json;

namespace compass_core {

// ---------- Сервисы /compass/config/* (JsonCall) ----------
//
// get_schema → схема; get → { version, values }; history → [ { version, createdAt, client, comment,
// changes } ]; apply { baseVersion, values, comment, client } и rollback { version, client } →
// { version, changed }. Ошибки проверки — INVALID_INPUT и { errors: { ключ: { code, message } } }.

void CompassCore::on_config(const std::string & op, JsonCall::Request::SharedPtr req, JsonCall::Response::SharedPtr res)
{
  const json in = json::parse(req->request_json.empty() ? "{}" : req->request_json, nullptr, false);
  if (in.is_discarded() || !in.is_object()) {
    return fail(res, "INVALID_INPUT", "request_json — объект JSON");
  }
  res->success = true;
  if (op == "get_schema") {
    res->response_json = config_->schema().dump();
    return;
  }
  if (op == "get") {
    res->response_json = json{{"version", config_->version()}, {"values", config_->values()}}.dump();
    return;
  }
  if (op == "history") {
    res->response_json = config_->history().dump();
    return;
  }
  const std::string client = in.value("client", "");
  const auto r = op == "apply"
                   ? config_->apply(in.value("baseVersion", -1), in.value("values", json::object()),
                                    in.value("comment", ""), client)
                   : config_->rollback(in.value("version", -1), client);
  if (!r.ok) {
    fail(res, r.code.c_str(), r.message);
    res->response_json = json{{"errors", r.errors}}.dump();
    return;
  }
  res->response_json = json{{"version", config_->version()}, {"changed", r.changed}}.dump();
  if (r.changed.empty()) {
    return;
  }
  apply_config(r.changed);
  robot_interfaces::msg::ConfigChanged msg;
  msg.version = config_->version();
  for (const auto & key : r.changed) {
    msg.changed_keys.push_back(key);
  }
  config_changed_pub_->publish(msg);
  emit_event("info", "CONFIG_APPLIED", "Config version " + std::to_string(config_->version()),
             {{"version", std::to_string(config_->version())}, {"client", client}});
  RCLCPP_INFO(get_logger(), "Настройки: версия %d (%s)", config_->version(), r.changed.dump().c_str());
}

// ---------- Применение ----------

void CompassCore::apply_config(const json & changed)
{
  const auto has = [&changed](const std::string & prefix) {
    if (changed.is_null()) {
      return true;
    }
    for (const auto & key : changed) {
      if (key.get<std::string>().rfind(prefix, 0) == 0) {
        return true;
      }
    }
    return false;
  };
  const json & v = config_->values();
  if (has("identity.")) {
    robot_name_ = v["identity.name"];
    serial_number_ = v["identity.serial_number"];
    manufacturer_ = v["identity.manufacturer"];
    model_ = v["identity.model"];
  }
  if (has("motion.")) {
    max_linear_ = setting("motion.max_linear");  // джойстик — сразу
    max_angular_ = setting("motion.max_angular");
    if (navigation_up_) {
      push_nav_limits();
    }
  }
  if (has("battery.")) {
    battery_low_ = battery_critical_ = false;  // пороги сменились — проверим заново
    check_battery();
  }
}

// Лимиты скорости Nav2 — в velocity_smoother: он стоит после любого контроллера (MPPI, DWB, …),
// и его параметры меняются на ходу.
void CompassCore::push_nav_limits()
{
  if (!smoother_params_->service_is_ready()) {
    return;
  }
  // Вбок — тот же предел, что вперёд; у неголономного робота y всегда 0.
  const double lin = max_linear_, lat = holonomic_ ? max_linear_ : 0.0, ang = max_angular_;
  smoother_params_->set_parameters({
    rclcpp::Parameter("max_velocity", std::vector<double>{lin, lat, ang}),
    rclcpp::Parameter("min_velocity", std::vector<double>{-lin, -lat, -ang}),
  });
}

// Пороги батареи; гистерезис 2 % — чтобы событие не повторялось, пока заряд колеблется у порога.
void CompassCore::check_battery()
{
  if (!battery_.present) {
    return;
  }
  const double p = battery_.percent;
  const std::string pct = std::to_string(static_cast<int>(p));
  if (!battery_low_ && p < setting("battery.warn_percent")) {
    battery_low_ = true;
    emit_event("warning", "BATTERY_LOW", "Battery low: " + pct + "%", {{"percent", pct}});
  } else if (battery_low_ && p > setting("battery.warn_percent") + 2) {
    battery_low_ = false;
  }
  if (!battery_critical_ && p < setting("battery.critical_percent")) {
    battery_critical_ = true;
    if (nav_.source == "compass") {
      cancel_goal();  // задание fms не трогаем: возможно, fms как раз везёт робота на зарядку
    }
    emit_event("error", "BATTERY_CRITICAL", "Battery critical: " + pct + "%", {{"percent", pct}});
  } else if (battery_critical_ && p > setting("battery.critical_percent") + 2) {
    battery_critical_ = false;
  }
}

}  // namespace compass_core
