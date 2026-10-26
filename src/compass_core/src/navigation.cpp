#include "compass_core/compass_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <type_traits>

using namespace std::chrono_literals;

namespace compass_core {

namespace {

// NavStart отвечает, когда Nav2 принял цель. Если перед ней отменяется прежняя или Nav2 не отвечает —
// ждём не дольше этого, иначе UI повис бы на «Ставим цель».
constexpr auto kNavStartReply = 5s;

// Новую цель после отменённой прежней отправляем не сразу: follow_waypoints отвечает «отменено»
// раньше, чем bt_navigator закончил свою navigate_to_pose, и цель, пришедшая в эти миллисекунды,
// прерывается. check_nav отправит её через kNextGoalDelay..kNextGoalDelay + 500 мс.
constexpr auto kNextGoalDelay = 300ms;

// Пороги локализации по σ положения и курса из ковариации AMCL.
constexpr double kOkSigma = 0.6;    // м и рад: робот знает, где он
constexpr double kLostSigma = 2.0;  // м: потерялся

std::string make_goal_id()
{
  static std::mt19937_64 rng{std::random_device{}()};
  char buf[33];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx",
                static_cast<unsigned long long>(rng()), static_cast<unsigned long long>(rng()));
  return buf;
}

// stamp нулевой — Nav2 возьмёт последний TF.
geometry_msgs::msg::PoseStamped pose_stamped(const std::string & frame, const robot_interfaces::msg::Pose2D & p)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame;
  msg.pose.position.x = p.x;
  msg.pose.position.y = p.y;
  msg.pose.orientation.z = std::sin(p.theta / 2);
  msg.pose.orientation.w = std::cos(p.theta / 2);
  return msg;
}

// Код ошибки Nav2 → NavStatus.error_code. Сотни — какой сервер не справился (nav2_msgs/action):
// 2xx, 3xx — планировщик (нет пути, цель в препятствии или вне карты), 1xx — контроллер
// (нет прогресса, нет допустимой скорости), 7xx — восстановления (spin, backup).
// bt_navigator передаёт их, если они перечислены в error_code_names (nav2.yaml).
std::string nav_error(uint16_t code)
{
  switch (code / 100) {
    case 2: case 3: return "NO_PATH";
    case 1: case 7: return "STUCK";
    default: return "NAV_FAILED";
  }
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
// Цель Compass — прямо в Nav2 (ROS_INTERFACES.md §4): goal → navigate_to_pose, маршрут →
// follow_waypoints или navigate_through_poses. NavStart отвечает, когда Nav2 принял цель.
// Новая цель при активной: прежнюю отменяем и отправляем новую, когда Nav2 её завершил —
// navigate_to_pose и navigate_through_poses делят один bt_navigator, и пока работает одна,
// другую он отклоняет.

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

  nav_start_id_ = id;
  nav_start_deadline_ = ChildProcess::Clock::now() + kNavStartReply;
  if (nav_.state == "active") {
    cancel_goal();
    nav_next_ = req;  // отправит check_nav, когда Nav2 завершит прежнюю
    return;
  }
  send_goal(*req);
}

void CompassCore::send_goal(const NavStart::Request & req)
{
  const bool goal = req.kind == "goal";
  const bool waypoints = req.mode == "waypoints";
  const bool ready = goal ? to_pose_client_->action_server_is_ready()
                     : waypoints ? waypoints_client_->action_server_is_ready()
                     : through_poses_client_->action_server_is_ready();
  if (!ready) {
    return reply_nav_start(false, "INTERNAL", "Nav2 не отвечает");
  }

  prev_nav_ = nav_;
  nav_ = robot_interfaces::msg::NavStatus{};
  nav_.goal_id = make_goal_id();
  nav_.source = "compass";
  nav_.operator_name = lease_.holder_name();
  nav_.kind = req.kind;
  nav_.mode = goal ? "" : req.mode;
  nav_.poses = req.poses;
  nav_.state = "active";
  nav_.distance_remaining = -1.0f;
  nav_.eta = -1.0f;

  if (goal) {
    NavigateToPose::Goal g;
    g.pose = pose_stamped(map_frame_, req.poses[0]);
    send_nav<NavigateToPose>(to_pose_client_, g);
  } else if (waypoints) {
    FollowWaypoints::Goal g;
    for (const auto & p : req.poses) {
      g.poses.push_back(pose_stamped(map_frame_, p));
    }
    send_nav<FollowWaypoints>(waypoints_client_, g);
  } else {
    NavigateThroughPoses::Goal g;
    for (const auto & p : req.poses) {
      g.poses.push_back(pose_stamped(map_frame_, p));
    }
    send_nav<NavigateThroughPoses>(through_poses_client_, g);
  }
  RCLCPP_INFO(get_logger(), "Цель %s: %s %s, точек %zu", nav_.goal_id.c_str(), nav_.kind.c_str(),
              nav_.mode.c_str(), nav_.poses.size());
}

// Ответы Nav2 сверяем с goal_id: результат прежней или брошенной цели не должен трогать текущую.
template <typename Action>
void CompassCore::send_nav(const std::shared_ptr<rclcpp_action::Client<Action>> & client,
                           const typename Action::Goal & goal)
{
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  typename rclcpp_action::Client<Action>::SendGoalOptions o;
  const std::string id = nav_.goal_id;

  o.goal_response_callback = [this, client, id](typename Handle::SharedPtr h) {
    if (id != nav_.goal_id) {
      if (h) {
        client->async_cancel_goal(h);  // NavStart уже ответил отказом по таймауту — цель не нужна
      }
      return;
    }
    if (!h) {
      nav_ = prev_nav_;
      return reply_nav_start(false, "INTERNAL", "Nav2 не принял цель");
    }
    nav_cancel_ = [client, h] { client->async_cancel_goal(h); };
    reply_nav_start(true, "", "");
    if (nav_cancel_wanted_) {
      nav_cancel_();  // отмену попросили, пока Nav2 принимал цель
    }
  };

  o.feedback_callback = [this, id](typename Handle::SharedPtr, const std::shared_ptr<const typename Action::Feedback> fb) {
    if (id != nav_.goal_id || nav_.state != "active") {
      return;
    }
    if constexpr (std::is_same_v<Action, FollowWaypoints>) {
      nav_.index = fb->current_waypoint;  // расстояния follow_waypoints не сообщает
    } else {
      nav_.distance_remaining = fb->distance_remaining;
      nav_.eta = static_cast<float>(rclcpp::Duration(fb->estimated_time_remaining).seconds());
      nav_.recoveries = static_cast<uint16_t>(fb->number_of_recoveries);
    }
    if constexpr (std::is_same_v<Action, NavigateThroughPoses>) {
      const auto total = nav_.poses.size();
      const auto remaining = std::min<std::size_t>(std::max<int>(fb->number_of_poses_remaining, 0), total);
      nav_.index = static_cast<uint32_t>(total - remaining);
    }
    nav_dirty_ = true;
  };

  o.result_callback = [this, id](const typename Handle::WrappedResult & w) {
    if (id != nav_.goal_id || nav_.state != "active") {
      return;
    }
    const uint16_t error = w.result ? w.result->error_code : 0;
    const std::string message = w.result ? w.result->error_msg : "";
    std::string missed;  // follow_waypoints завершается успешно, даже если часть точек пропущена
    if constexpr (std::is_same_v<Action, FollowWaypoints>) {
      if (w.result) {
        for (const auto & m : w.result->missed_waypoints) {
          missed += (missed.empty() ? "" : ", ") + std::to_string(m.index + 1);
        }
      }
    }
    finish_goal(w.code, error, message, missed);
  };

  client->async_send_goal(goal, o);
}

void CompassCore::finish_goal(rclcpp_action::ResultCode code, uint16_t error, const std::string & message,
                              const std::string & missed)
{
  nav_cancel_ = nullptr;
  nav_cancel_wanted_ = false;
  if (code == rclcpp_action::ResultCode::SUCCEEDED) {
    nav_.state = "succeeded";
    nav_.index = static_cast<uint32_t>(nav_.poses.size());
    nav_.distance_remaining = 0.0f;
    if (!missed.empty()) {
      emit_event("warning", "NAV_WAYPOINTS_MISSED", "Waypoints missed: " + missed, {{"points", missed}});
    } else {
      emit_event("info", "NAV_GOAL_SUCCEEDED", "Goal reached");
    }
  } else if (code == rclcpp_action::ResultCode::CANCELED) {
    nav_.state = "canceled";
    emit_event("info", "NAV_GOAL_CANCELED", "Goal canceled");
  } else {
    nav_.state = "failed";
    nav_.error_code = nav_error(error);
    // Текст Nav2 в Jazzy обычно пустой — тогда хотя бы код.
    nav_.error_message = message.empty() ? "Nav2 error " + std::to_string(error) : message;
    emit_event("warning", "NAV_GOAL_FAILED", "Goal failed: " + nav_.error_code + ", " + nav_.error_message,
               {{"code", nav_.error_code}, {"nav2_code", std::to_string(error)}});
  }
  RCLCPP_INFO(get_logger(), "Цель %s: %s %u %s", nav_.goal_id.c_str(), nav_.state.c_str(), error, message.c_str());
  publish_nav_status();
  nav_next_at_ = ChildProcess::Clock::now() + kNextGoalDelay;
}

void CompassCore::on_nav_cancel(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  cancel_goal();
  res->success = true;
}

// Итог отмены (canceled) приходит результатом Nav2 — finish_goal.
void CompassCore::cancel_goal()
{
  if (nav_next_) {  // новая цель ждала, пока отменится прежняя, — отменяется и она
    nav_next_.reset();
    reply_nav_start(false, "BUSY", "цель отменена до начала");
  }
  if (nav_.state != "active") {
    return;
  }
  if (nav_cancel_) {
    nav_cancel_();
  } else {
    nav_cancel_wanted_ = true;
  }
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
  const auto now = ChildProcess::Clock::now();
  if (nav_next_ && nav_.state != "active" && now >= nav_next_at_) {
    const auto req = nav_next_;
    nav_next_.reset();
    send_goal(*req);
  }
  if (nav_start_id_ && now > nav_start_deadline_) {
    if (nav_next_) {
      nav_next_.reset();
      reply_nav_start(false, "BUSY", "цель не началась: прежняя не отменилась вовремя");
    } else {
      nav_ = prev_nav_;  // ответ Nav2, если придёт позже, отменит цель (goal_id уже другой)
      nav_cancel_wanted_ = false;
      reply_nav_start(false, "INTERNAL", "Nav2 не ответил на цель");
    }
  }
  // Nav2 остановлен (смена карты, падение стека) — его результата уже не будет.
  if (nav_.state == "active" && !navigation_up_ && !nav_start_id_) {
    finish_goal(rclcpp_action::ResultCode::CANCELED, 0, "", "");
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

}  // namespace compass_core
