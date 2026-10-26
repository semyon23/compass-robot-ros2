#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Езда по рёбрам order'а: action FollowPath прямо в controller_server Nav2.
// Планировщик Nav2 не участвует — робот едет по ломаной через узлы, без объездов.
class Drive {
public:
  struct Point {
    double x;
    double y;
  };
  enum class Result { None, Succeeded, Failed };

  explicit Drive(rclcpp::Node & node);

  // Ехать от позы робота через points (фрейм map). Если уже едем, новый путь заменяет старый
  // без остановки: controller_server принимает новую цель на ходу.
  void follow(double x, double y, const std::vector<Point> & points);
  void stop();
  bool active() const { return active_; }

  // Чем закончилась последняя цель; один раз, потом None.
  Result take_result() { return std::exchange(result_, Result::None); }

private:
  using FollowPath = nav2_msgs::action::FollowPath;

  rclcpp::Node & node_;
  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr goal_;
  std::uint64_t goal_number_ = 0;  // ответы на заменённые и отменённые цели не учитываем
  bool active_ = false;
  Result result_ = Result::None;
};
