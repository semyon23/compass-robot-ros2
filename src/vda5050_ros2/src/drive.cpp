#include "vda5050_ros2/drive.hpp"

#include <cmath>

namespace
{
// Вершины ломаной: compass_core кладёт в /plan точки через 5 см, лишние — те, что на одной прямой
// с соседями и в том же направлении (разворот назад — вершина).
std::vector<Drive::Point> corners(const nav_msgs::msg::Path & path)
{
  std::vector<Drive::Point> out;
  for (const auto & pose : path.poses) {
    const Drive::Point p{pose.pose.position.x, pose.pose.position.y};
    if (out.size() >= 2) {
      const auto & a = out[out.size() - 2];
      const auto & b = out.back();
      const double cross = (b.x - a.x) * (p.y - b.y) - (b.y - a.y) * (p.x - b.x);
      const double dot = (b.x - a.x) * (p.x - b.x) + (b.y - a.y) * (p.y - b.y);
      if (std::abs(cross) < 1e-6 && dot > 0) {
        out.back() = p;
        continue;
      }
    }
    out.push_back(p);
  }
  return out;
}
}  // namespace

Drive::Drive(rclcpp::Node & node)
: node_(node),
  start_client_(node.create_client<NavStart>("/compass/nav/start")),
  cancel_client_(node.create_client<std_srvs::srv::Trigger>("/compass/nav/cancel"))
{
  status_sub_ = node.create_subscription<NavStatus>(
    "/compass/nav_status", rclcpp::QoS(1).reliable().transient_local(), [this](const NavStatus & msg) {
      last_status_ = msg;
      check_status();
    });
  // /plan публикует compass_core при отправке маршрута, каждой его части и повторе после препятствия.
  // Пока едет наша цель, других целей у compass_core нет — путь наш.
  plan_sub_ = node.create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(1), [this](const nav_msgs::msg::Path & msg) {
      if (active_) {
        plan_ = corners(msg);
      }
    });
}

bool Drive::follow(double x, double y, const std::vector<Point> & points)
{
  if (pending_) {
    return false;
  }
  if (!start_client_->service_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "compass_core (/compass/nav/start) is not available");
    active_ = false;
    result_ = Result::Failed;
    return true;
  }

  auto req = std::make_shared<NavStart::Request>();
  req->source = "fms";
  req->kind = "route";
  req->mode = "line";
  // Курс в узле — вдоль подъезда к нему: робот приходит в конец отрезка, не разворачиваясь.
  Point from{x, y};
  for (const auto & p : points) {
    robot_interfaces::msg::Pose2D pose;
    pose.x = p.x;
    pose.y = p.y;
    pose.theta = std::atan2(p.y - from.y, p.x - from.x);
    req->poses.push_back(pose);
    from = p;
  }

  const std::uint64_t number = ++request_number_;
  pending_ = true;
  active_ = true;
  goal_id_.clear();
  plan_.clear();  // прежний путь больше не наш; новый придёт в /plan до ответа compass_core
  start_client_->async_send_request(req, [this, number](rclcpp::Client<NavStart>::SharedFuture future) {
    pending_ = false;
    const auto res = future.get();
    if (number != request_number_) {
      if (res->success) {  // stop() пришёл раньше ответа — цель уже не нужна
        cancel_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      }
      return;
    }
    if (!res->success) {
      // BUSY — Nav2 поднимается или прежняя цель ещё отменяется: drive_step() повторит на следующем тике.
      RCLCPP_WARN(node_.get_logger(), "compass_core refused the route: %s %s", res->error_code.c_str(),
                  res->message.c_str());
      if (res->error_code != "BUSY") {
        cancel_fms();  // продление не приняли — прежний отрезок не доезжаем, агент ждёт fms
      }
      active_ = false;
      result_ = res->error_code == "BUSY" ? Result::None : Result::Failed;
      return;
    }
    goal_id_ = res->goal_id;
    check_status();  // короткий отрезок мог закончиться раньше, чем пришёл ответ
  });
  return true;
}

void Drive::check_status()
{
  if (!active_ || goal_id_.empty() || last_status_.goal_id != goal_id_ || last_status_.state == "active") {
    return;
  }
  active_ = false;
  goal_id_.clear();
  if (last_status_.state == "succeeded") {
    result_ = Result::Succeeded;
  } else if (last_status_.state == "failed") {
    RCLCPP_WARN(node_.get_logger(), "route failed: %s %s", last_status_.error_code.c_str(),
                last_status_.error_message.c_str());
    result_ = Result::Failed;
  }
}

void Drive::stop()
{
  ++request_number_;
  cancel_fms();
  active_ = false;
  goal_id_.clear();
}

// /compass/nav/cancel отменяет любую цель — зовём, только если у compass_core идёт цель fms.
void Drive::cancel_fms()
{
  if (last_status_.source == "fms" && last_status_.state == "active") {
    cancel_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }
}
