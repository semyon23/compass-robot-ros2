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

// Маршрут «по линии»: шаг точек пути (контроллеру нужен плотный путь); точка засчитана, когда
// робот проехал вдоль линии до неё без этого; сколько раз перезапускать часть, если контроллер
// упёрся (препятствие на линии): progress_checker сдаётся за ~10 с — итого робот ждёт около минуты.
constexpr double kPathStep = 0.05;
constexpr double kLinePassed = 0.15;
constexpr int kLineRetries = 6;

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

// Где резать маршрут «по линии» на части: индексы последних точек частей. MPPI и goal_checker
// в Jazzy меряют расстояние до конца пути по прямой: если с какого-то места линии конец ближе gap
// по прямой, а по линии до него заметно дальше (петля, разворот), робот поехал бы прямо к концу.
// Поэтому часть кончается в точке, после которой такое место появилось бы. «Заметно» — путь длиннее
// прямой больше чем на gap / 2: на обычных поворотах ломаной разница мала, и линию не режем.
std::vector<std::size_t> split_line(double x0, double y0, const std::vector<LinePoint> & line, double gap)
{
  std::vector<std::size_t> breaks;
  std::vector<LinePoint> dense{{x0, y0, 0.0}};  // плотные точки текущей части
  for (std::size_t k = 0; k < line.size(); ++k) {
    const LinePoint & p = line[k];
    const bool loop = std::any_of(dense.begin(), dense.end(), [&](const LinePoint & d) {
      const double straight = std::hypot(p.x - d.x, p.y - d.y);
      return straight < gap && p.s - d.s - straight > gap / 2;
    });
    if (loop && k > 0 && (breaks.empty() || breaks.back() != k - 1)) {
      breaks.push_back(k - 1);
      dense = {line[k - 1]};
    }
    const LinePoint from = dense.back();
    const int steps = std::max(1, static_cast<int>(std::ceil((p.s - from.s) / kPathStep)));
    for (int i = 1; i <= steps; ++i) {
      const double f = static_cast<double>(i) / steps;
      dense.push_back({from.x + (p.x - from.x) * f, from.y + (p.y - from.y) * f, from.s + (p.s - from.s) * f});
    }
  }
  breaks.push_back(line.size() - 1);
  return breaks;
}

// Отрезок a → b точками через kPathStep (без a, если путь не пуст), ориентация — вдоль отрезка.
void append_segment(nav_msgs::msg::Path & path, double ax, double ay, double bx, double by)
{
  const double yaw = std::atan2(by - ay, bx - ax);
  const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(bx - ax, by - ay) / kPathStep)));
  for (int i = path.poses.empty() ? 0 : 1; i <= steps; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = ax + (bx - ax) * i / steps;
    p.pose.position.y = ay + (by - ay) * i / steps;
    p.pose.orientation.z = std::sin(yaw / 2);
    p.pose.orientation.w = std::cos(yaw / 2);
    path.poses.push_back(p);
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
// Цель Compass — прямо в Nav2 (ROS_INTERFACES.md §4): goal → navigate_to_pose, маршрут
// «по точкам» → follow_waypoints, маршрут «по линии» → follow_path контроллера: ломаная через точки,
// без планировщика — робот едет по нарисованной линии. NavStart отвечает, когда Nav2 принял цель.
// Новая цель при активной: прежнюю отменяем и отправляем новую, когда Nav2 её завершил.

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
  const bool route = req->kind == "route" && (req->mode == "waypoints" || req->mode == "line");
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
                     : follow_path_client_->action_server_is_ready();
  if (!ready) {
    return reply_nav_start(false, "INTERNAL", "Nav2 не отвечает");
  }
  double x = 0.0, y = 0.0;  // линия начинается от робота
  if (!goal && !waypoints && !robot_in_map(x, y)) {
    return reply_nav_start(false, "INTERNAL", "нет позы робота на карте");
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
    line_.clear();
    double s = 0.0, px = x, py = y;
    for (const auto & p : req.poses) {
      s += std::hypot(p.x - px, p.y - py);
      line_.push_back({p.x, p.y, s});
      px = p.x;
      py = p.y;
    }
    line_breaks_ = split_line(x, y, line_, route_loop_gap_);
    line_x0_ = x;
    line_y0_ = y;
    line_s_ = 0.0;
    line_part_ = 0;
    line_retries_ = 0;
    RCLCPP_INFO(get_logger(), "Линия %.1f м, частей %zu", s, line_breaks_.size());
    send_line_part();
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
    if (!h && nav_start_id_) {
      nav_ = prev_nav_;
      return reply_nav_start(false, "INTERNAL", "Nav2 не принял цель");
    }
    if (!h) {  // следующую часть маршрута «по линии» — цель уже идёт, NavStart давно ответил
      return finish_goal(rclcpp_action::ResultCode::ABORTED, 0, "Nav2 не принял часть маршрута", "");
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
    } else if constexpr (std::is_same_v<Action, FollowPath>) {
      // Прогресс считаем сами — проекцией позы на линию: distance_to_goal контроллера в начале
      // завышен, на поворотах замирает (nav_run11). Пройдены точки, до которых доехали по линии.
      double x = 0.0, y = 0.0;
      if (robot_in_map(x, y)) {
        line_s_ = line_progress(x, y);
      }
      while (nav_.index < line_.size() && line_[nav_.index].s <= line_s_ + kLinePassed) {
        ++nav_.index;
      }
      const double remaining = line_.back().s - line_s_;
      nav_.distance_remaining = static_cast<float>(remaining);
      nav_.eta = fb->speed > 0.05f ? static_cast<float>(remaining / fb->speed) : -1.0f;
    } else {
      nav_.distance_remaining = fb->distance_remaining;
      nav_.eta = static_cast<float>(rclcpp::Duration(fb->estimated_time_remaining).seconds());
      nav_.recoveries = static_cast<uint16_t>(fb->number_of_recoveries);
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
    if constexpr (std::is_same_v<Action, FollowPath>) {
      on_line_result(w.code, error, message);
    } else {
      finish_goal(w.code, error, message, missed);
    }
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

bool CompassCore::robot_in_map(double & x, double & y)
{
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

// Проекция позы робота на линию — на отрезки, что лежат в окне −0.3…+1.0 м по линии от прошлого
// положения: на петле, где линия проходит рядом сама с собой, не перескочит на другой её участок.
double CompassCore::line_progress(double x, double y) const
{
  const double lo = line_s_ - 0.3, hi = line_s_ + 1.0;
  double best_d = 1e9, best_s = line_s_;
  double ax = line_x0_, ay = line_y0_, as = 0.0;
  for (const LinePoint & b : line_) {
    if (b.s >= lo && as <= hi && b.s > as) {
      const double dx = b.x - ax, dy = b.y - ay;
      const double t = std::clamp(((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
      const double s = std::clamp(as + t * (b.s - as), lo, hi);
      const double d = std::hypot(x - (ax + t * dx), y - (ay + t * dy));
      if (d < best_d) {
        best_d = d;
        best_s = s;
      }
    }
    ax = b.x;
    ay = b.y;
    as = b.s;
  }
  return std::max(0.0, best_s);
}

// Путь текущей части: от позы робота через непройденные точки части; в конце маршрута — курс
// последней точки. Перезапуск части тоже строится заново от робота: контроллер ищет ближайшую к
// роботу точку только в начале пути, и старый путь повёл бы его назад.
void CompassCore::send_line_part()
{
  double x = 0.0, y = 0.0;
  if (!robot_in_map(x, y)) {
    return finish_goal(rclcpp_action::ResultCode::ABORTED, 0, "нет позы робота на карте", "");
  }
  const std::size_t first = line_part_ == 0 ? 0 : line_breaks_[line_part_ - 1] + 1;
  const std::size_t last = line_breaks_[line_part_];
  FollowPath::Goal g;
  g.path.header.frame_id = map_frame_;  // stamp = 0: контроллер берёт последний TF
  for (std::size_t k = std::min<std::size_t>(std::max<std::size_t>(first, nav_.index), last); k <= last; ++k) {
    append_segment(g.path, x, y, line_[k].x, line_[k].y);
    x = line_[k].x;
    y = line_[k].y;
  }
  if (last + 1 == line_.size()) {
    const double theta = nav_.poses.back().theta;
    g.path.poses.back().pose.orientation.z = std::sin(theta / 2);
    g.path.poses.back().pose.orientation.w = std::cos(theta / 2);
  }
  plan_pub_->publish(g.path);  // планировщик не участвует — путь на карте Compass рисуем сами
  send_nav<FollowPath>(follow_path_client_, g);
}

// Часть доехана — следующая. Контроллер упёрся (нет прогресса, нет допустимой скорости) — чаще всего
// препятствие на линии: ждём, перезапуская остаток части, пока не кончатся попытки.
void CompassCore::on_line_result(rclcpp_action::ResultCode code, uint16_t error, const std::string & message)
{
  nav_cancel_ = nullptr;  // прежней части больше нет; отмену до приёма новой запомнит cancel_goal
  if (code == rclcpp_action::ResultCode::SUCCEEDED && line_part_ + 1 < line_breaks_.size()) {
    ++line_part_;
    line_retries_ = 0;
    return send_line_part();
  }
  const bool stuck = error == FollowPath::Result::PATIENCE_EXCEEDED ||
                     error == FollowPath::Result::FAILED_TO_MAKE_PROGRESS ||
                     error == FollowPath::Result::NO_VALID_CONTROL;
  if (code == rclcpp_action::ResultCode::ABORTED && stuck && line_retries_ < kLineRetries) {
    ++line_retries_;
    nav_.recoveries = static_cast<uint16_t>(line_retries_);
    RCLCPP_INFO(get_logger(), "Линия: контроллер упёрся (%u), попытка %d", error, line_retries_);
    return send_line_part();
  }
  finish_goal(code, error, message, "");
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
