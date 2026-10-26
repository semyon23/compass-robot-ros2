#include "vda5050_ros2/drive.hpp"

#include <cmath>
#include <utility>

namespace
{
constexpr double kPathStep = 0.05;  // [м] шаг точек пути: контроллеру нужен плотный путь
}

Drive::Drive(rclcpp::Node & node)
: node_(node),
  client_(rclcpp_action::create_client<FollowPath>(&node, "follow_path"))
{
}

void Drive::follow(double x, double y, const std::vector<Point> & points)
{
  // Ломаная робот → узлы, точки через kPathStep; ориентация — вдоль отрезка.
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";  // stamp = 0: контроллер берёт последний TF
  Point from{x, y};
  for (const auto & to : points) {
    const double length = std::hypot(to.x - from.x, to.y - from.y);
    const double yaw = std::atan2(to.y - from.y, to.x - from.x);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / kPathStep)));
    for (int i = path.poses.empty() ? 0 : 1; i <= steps; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = from.x + (to.x - from.x) * i / steps;
      p.pose.position.y = from.y + (to.y - from.y) * i / steps;
      p.pose.orientation.z = std::sin(yaw / 2);
      p.pose.orientation.w = std::cos(yaw / 2);
      path.poses.push_back(p);
    }
    from = to;
  }

  if (!client_->action_server_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "controller_server (follow_path) is not available");
    result_ = Result::Failed;
    active_ = false;
    return;
  }

  FollowPath::Goal goal;
  goal.path = path;
  const std::uint64_t number = ++goal_number_;
  active_ = true;
  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.goal_response_callback = [this, number](const auto & handle) {
    if (number != goal_number_) {
      if (handle) {
        client_->async_cancel_goal(handle);  // stop() пришёл раньше, чем ответ на цель
      }
      return;
    }
    if (!handle) {
      active_ = false;
      result_ = Result::Failed;
      return;
    }
    goal_ = handle;
  };
  options.result_callback = [this, number](const auto & wrapped) {
    if (number != goal_number_) {
      return;  // цель заменили новой или отменили
    }
    active_ = false;
    goal_.reset();
    result_ = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED ? Result::Succeeded : Result::Failed;
  };
  client_->async_send_goal(goal, options);
}

void Drive::stop()
{
  ++goal_number_;
  active_ = false;
  if (goal_) {
    client_->async_cancel_goal(goal_);
    goal_.reset();
  }
}
