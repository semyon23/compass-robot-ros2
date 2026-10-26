#include "compass_core/compass_core.hpp"

#include <cmath>
#include <cstdio>
#include <random>

#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace compass_core {

namespace {

// Новая цель при активной ждёт в очереди оркестратора, пока прежняя отменяется. Очередь ждёт и
// допуска (заряд, режим) — поэтому их compass_core проверяет сам заранее, а цель, которая так и
// не началась за kNavStartReply, отменяет: иначе она могла бы поехать сама намного позже.
// deadline_at оркестратора не подходит — он ограничивает и время самой езды.
constexpr auto kNavStartReply = 6s;

// Пороги локализации по σ положения и курса из ковариации AMCL.
constexpr double kOkSigma = 0.6;    // м и рад: робот знает, где он
constexpr double kLostSigma = 2.0;  // м: потерялся

std::string make_task_id()
{
  static std::mt19937_64 rng{std::random_device{}()};
  char buf[33];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx",
                static_cast<unsigned long long>(rng()), static_cast<unsigned long long>(rng()));
  return buf;
}

// Pose2D → geometry_msgs/PoseStamped в JSON (как ждёт set_message_fields оркестратора).
// stamp нулевой — Nav2 возьмёт последний TF.
json pose_stamped(const robot_interfaces::msg::Pose2D & p)
{
  return {
    {"header", {{"frame_id", "map"}}},
    {"pose", {
      {"position", {{"x", p.x}, {"y", p.y}, {"z", 0.0}}},
      {"orientation", {{"x", 0.0}, {"y", 0.0}, {"z", std::sin(p.theta / 2)}, {"w", std::cos(p.theta / 2)}}},
    }},
  };
}

}  // namespace

// ---------- Локализация ----------

void CompassCore::on_amcl_pose(const PoseCov & msg)
{
  if (stack_name_ != "nav" || stack_state_ != "running" || restart_stack_) {
    return;  // restart_stack_ — это поза старого AMCL, карта уже другая
  }
  const auto & c = msg.pose.covariance;
  const double sxy = std::sqrt(std::max(c[0], c[7]));
  const double syaw = std::sqrt(c[35]);
  const std::string status = sxy < kOkSigma && syaw < kOkSigma ? "ok" : sxy < kLostSigma ? "uncertain" : "lost";
  const bool changed = status != localization_.status;
  localization_.status = status;
  localization_.covariance = static_cast<float>(sxy);
  if (changed) {
    if (status == "lost") {
      emit_event("warning", "LOCALIZATION_LOST", "Localization lost");
    }
    publish_status();
  }
}

// Начальная поза для AMCL — как «2D Pose Estimate» в RViz.
void CompassCore::publish_initial_pose(const robot_interfaces::msg::Pose2D & pose)
{
  PoseCov msg;
  msg.header.frame_id = map_frame_;
  // stamp нулевой: AMCL возьмёт последний TF. С now() он ругается «extrapolation into the future».
  msg.pose.pose.position.x = pose.x;
  msg.pose.pose.position.y = pose.y;
  msg.pose.pose.orientation.z = std::sin(pose.theta / 2);
  msg.pose.pose.orientation.w = std::cos(pose.theta / 2);
  msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.06853891909122467;
  initial_pose_pub_->publish(msg);
}

// ---------- Сервисные цели ----------
//
// Цель Compass — задача task_orchestrator (ROS_INTERFACES.md §4.4): action execute_task.
// Оркестратор принимает любой goal; отказ (допуск, конфликт, Nav2 не поднят) приходит
// результатом со status REJECTED. Поэтому NavStart отвечает по первому feedback (задача началась)
// или по результату (отказ или мгновенное завершение); не началась за kNavStartReply — отмена.

void CompassCore::on_nav_start(RequestId id, NavStart::Request::SharedPtr req)
{
  auto res = std::make_shared<NavStart::Response>();
  auto reply = [this, id, res] { nav_start_srv_->send_response(*id, *res); };

  if (mode_ != "SERVICE") {
    fail(res, "MODE_NOT_ALLOWED", "сервисные цели — только в режиме SERVICE");
    return reply();
  }
  if (stopped_) {
    fail(res, "MODE_NOT_ALLOWED", "нажат СТОП — сначала снимите его");
    return reply();
  }
  if (!lease_.touch(req->lease_id, ControlLease::Clock::now())) {
    fail(res, "CONTROL_LOCKED", "сначала возьмите управление");
    return reply();
  }
  if (stack_name_ != "nav" || stack_state_ != "running") {
    fail(res, "MODE_NOT_ALLOWED", "навигация не запущена: выберите карту");
    return reply();
  }
  if (!localized()) {
    fail(res, "MODE_NOT_ALLOWED", "робот не локализован: задайте позу на карте");
    return reply();
  }
  // Допуск по заряду проверит и оркестратор, но с очередью — ждал бы; отвечаем сразу.
  if (battery_.present && battery_.percent < setting("battery.min_task_percent")) {
    fail(res, "MODE_NOT_ALLOWED", "заряд " + std::to_string(static_cast<int>(battery_.percent)) +
                                    "% ниже минимума для задач " +
                                    std::to_string(static_cast<int>(setting("battery.min_task_percent"))) + "%");
    return reply();
  }
  if (!navigation_up_) {
    fail(res, "BUSY", "Nav2 ещё поднимается");
    return reply();
  }
  const bool goal = req->kind == "goal";
  const bool route = req->kind == "route" && (req->mode == "waypoints" || req->mode == "through_poses");
  if ((goal && req->poses.size() != 1) || (!goal && !route) || req->poses.empty() || req->poses.size() > 100) {
    fail(res, "INVALID_INPUT", "цель — одна поза, маршрут — от 1 до 100 точек");
    return reply();
  }
  if (nav_start_id_) {
    fail(res, "BUSY", "предыдущая цель ещё ставится");
    return reply();
  }
  if (!task_client_->action_server_is_ready()) {
    fail(res, "INTERNAL", "task_orchestrator не запущен");
    return reply();
  }

  // Оркестратор цель не заменяет: прежнюю отменяем сами, новая ждёт в очереди (queue_on_conflict),
  // пока прежняя не освободит группу navigation.
  const bool replacing = nav_.state == "active";
  cancel_goal();

  json data;
  std::string task_name;
  if (goal) {
    task_name = "nav2/navigate_to_pose";
    data["pose"] = pose_stamped(req->poses[0]);
  } else {
    task_name = req->mode == "waypoints" ? "nav2/follow_waypoints" : "nav2/navigate_through_poses";
    data["poses"] = json::array();
    for (const auto & p : req->poses) {
      data["poses"].push_back(pose_stamped(p));
    }
  }

  ExecuteTask::Goal g;
  g.task_id = make_task_id();
  g.task_name = task_name;
  g.source = "compass";
  g.operator_id = lease_.holder_name();
  g.metadata_json = json{{"client_id", lease_.holder_id()}}.dump();
  g.task_data_json = data.dump();
  g.queue_on_conflict = replacing;

  prev_nav_ = nav_;
  nav_ = robot_interfaces::msg::NavStatus{};
  nav_.goal_id = g.task_id;
  nav_.source = "compass";
  nav_.operator_name = lease_.holder_name();
  nav_.kind = req->kind;
  nav_.mode = goal ? "" : req->mode;
  nav_.poses = req->poses;
  nav_.state = "active";
  nav_.distance_remaining = -1.0f;
  nav_.eta = -1.0f;
  nav_start_id_ = id;
  nav_start_deadline_ = ChildProcess::Clock::now() + kNavStartReply;

  rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
  options.goal_response_callback = [this, task_id = g.task_id](auto handle) {
    if (!handle && nav_.goal_id == task_id) {
      nav_ = prev_nav_;
      reply_nav_start(false, "INTERNAL", "task_orchestrator не принял задачу");
    }
  };
  options.feedback_callback = [this](auto, const std::shared_ptr<const ExecuteTask::Feedback> fb) {
    on_task_feedback(*fb);
  };
  options.result_callback = [this, task_id = g.task_id](const auto & wrapped) {
    if (wrapped.result) {
      on_task_result(task_id, *wrapped.result);
    }
  };
  task_client_->async_send_goal(g, options);
  RCLCPP_INFO(get_logger(), "Цель %s: %s, точек %zu", g.task_id.c_str(), task_name.c_str(), req->poses.size());
}

void CompassCore::on_nav_cancel(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  cancel_goal();
  res->success = true;
}

void CompassCore::cancel_goal()
{
  if (nav_.state != "active" || !cancel_tasks_client_->service_is_ready()) {
    return;
  }
  auto req = std::make_shared<CancelTasks::Request>();
  req->task_ids = {nav_.goal_id};
  req->source = "compass";
  cancel_tasks_client_->async_send_request(req);
}

void CompassCore::stop_tasks()
{
  if (!stop_tasks_client_->service_is_ready()) {
    return;
  }
  auto req = std::make_shared<StopTasks::Request>();
  req->source = "compass";
  stop_tasks_client_->async_send_request(req);
}

// Feedback Nav2 лежит в feedback_json.backend (поля — §4.4 контракта).
void CompassCore::on_task_feedback(const ExecuteTask::Feedback & fb)
{
  if (fb.task_id != nav_.goal_id || nav_.state != "active") {
    return;
  }
  reply_nav_start(true, "", "");  // задача началась
  const json j = json::parse(fb.feedback_json, nullptr, false);
  if (j.is_discarded() || !j.contains("backend")) {
    return;
  }
  const json & b = j["backend"];
  if (b.contains("distance_remaining")) {
    nav_.distance_remaining = b["distance_remaining"].get<float>();
  }
  if (b.contains("estimated_time_remaining")) {
    const json & t = b["estimated_time_remaining"];
    nav_.eta = t.value("sec", 0) + t.value("nanosec", 0) * 1e-9f;
  }
  if (b.contains("number_of_recoveries")) {
    nav_.recoveries = b["number_of_recoveries"].get<uint16_t>();
  }
  if (b.contains("current_waypoint")) {  // follow_waypoints
    nav_.index = b["current_waypoint"].get<uint32_t>();
  }
  if (b.contains("number_of_poses_remaining")) {  // navigate_through_poses
    const auto remaining = b["number_of_poses_remaining"].get<uint32_t>();
    nav_.index = static_cast<uint32_t>(nav_.poses.size()) - std::min<uint32_t>(remaining, nav_.poses.size());
  }
  nav_dirty_ = true;
}

void CompassCore::on_task_result(const std::string & task_id, const ExecuteTask::Result & r)
{
  if (task_id != nav_.goal_id) {
    return;  // результат прежней, уже заменённой цели
  }
  if (r.status == "REJECTED" && nav_start_id_) {
    // Отказ при постановке: допуск (режим, локализация), конфликт, Nav2 не поднят — ответ NavStart.
    const bool policy = r.error_code == "POLICY_REJECTED" || r.error_code == "RESOURCE_CONFLICT" ||
                        r.error_code == "SERVER_UNAVAILABLE";
    nav_ = prev_nav_;
    return reply_nav_start(false, policy ? "MODE_NOT_ALLOWED" : "INTERNAL", r.error_message);
  }
  reply_nav_start(true, "", "");

  if (r.status == "DONE") {
    nav_.state = "succeeded";
    nav_.index = static_cast<uint32_t>(nav_.poses.size());
    nav_.distance_remaining = 0.0f;
    // follow_waypoints завершается успешно, даже если часть точек пропущена.
    const json j = json::parse(r.result_json, nullptr, false);
    if (!j.is_discarded() && j.contains("missed_waypoints") && !j["missed_waypoints"].empty()) {
      std::string missed;
      for (const auto & w : j["missed_waypoints"]) {
        missed += (missed.empty() ? "" : ", ") + std::to_string(w.value("index", 0u) + 1);
      }
      emit_event("warning", "NAV_WAYPOINTS_MISSED", "Waypoints missed: " + missed, {{"points", missed}});
    } else {
      emit_event("info", "NAV_GOAL_SUCCEEDED", "Goal reached");
    }
  } else if (r.status == "CANCELED") {
    nav_.state = "canceled";
    emit_event("info", "NAV_GOAL_CANCELED", "Goal canceled");
  } else {
    // При неудаче Nav2 оркестратор 1.0.0 не передаёт его код ошибки — различаем только таймаут.
    nav_.state = "failed";
    const bool timeout = r.error_code == "TASK_TIMEOUT" || r.error_code == "DEADLINE_EXCEEDED";
    nav_.error_code = timeout ? "TIMEOUT" : "NAV_FAILED";
    nav_.error_message = r.error_message;
    emit_event("warning", "NAV_GOAL_FAILED", "Goal failed: " + r.error_message, {{"code", nav_.error_code}});
  }
  RCLCPP_INFO(get_logger(), "Цель %s: %s %s", task_id.c_str(), nav_.state.c_str(), r.error_message.c_str());
  publish_nav_status();
}

void CompassCore::reply_nav_start(bool success, const std::string & code, const std::string & message)
{
  if (!nav_start_id_) {
    return;
  }
  NavStart::Response res;
  res.success = success;
  res.error_code = code;
  res.message = message;
  res.goal_id = success ? nav_.goal_id : "";
  nav_start_srv_->send_response(*nav_start_id_, res);
  nav_start_id_.reset();
  if (success) {
    publish_nav_status();
  }
}

// Раз в 500 мс: прогресс активной цели (feedback Nav2 идёт чаще — UI столько не нужно).
void CompassCore::check_nav()
{
  if (nav_start_id_ && ChildProcess::Clock::now() > nav_start_deadline_) {
    cancel_goal();  // так и не началась — из очереди её убираем
    reply_nav_start(false, "BUSY", "цель не началась: прежняя не отменилась вовремя");
  }
  if (nav_dirty_ && nav_.state == "active") {
    publish_nav_status();
  }
}

void CompassCore::publish_nav_status()
{
  nav_status_pub_->publish(nav_);
  nav_dirty_ = false;
}

// Допуск задач (§4.3): оркестратор сам решает, можно ли начать задачу, по этим параметрам.
// Шлём при изменении и раз в 5 с — на случай, если оркестратор перезапускался.
void CompassCore::update_admission()
{
  if (!orchestrator_params_->service_is_ready()) {
    return;
  }
  const std::string robot_mode = state_ == "switching" ? "SWITCHING" : mode_;
  const double battery = battery_.present ? battery_.percent : 100.0;
  const std::string key = robot_mode + "|" + (localized() ? "1" : "0") + "|" + std::to_string(battery) + "|" +
                          (stopped_ ? "1" : "0");
  const auto now = ChildProcess::Clock::now();
  if (key == admission_sent_ && now - admission_at_ < 5s) {
    return;
  }
  admission_sent_ = key;
  admission_at_ = now;
  orchestrator_params_->set_parameters({
    rclcpp::Parameter("admission.robot_mode", robot_mode),
    rclcpp::Parameter("admission.localization_ok", localized()),
    rclcpp::Parameter("admission.emergency_stop_active", stopped_),
    rclcpp::Parameter("admission.battery_percent", battery),
  });
}

}  // namespace compass_core
