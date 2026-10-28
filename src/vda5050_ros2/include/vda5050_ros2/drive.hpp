#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/nav_status.hpp>
#include <robot_interfaces/srv/nav_start.hpp>
#include <std_srvs/srv/trigger.hpp>

// Езда по рёбрам order'а — через compass_core, как маршрут «по линии» из Compass:
// /compass/nav/start (source fms), итог — /compass/nav_status, отмена — /compass/nav/cancel.
// Nav2 ведёт compass_core: ломаная через узлы, нарезка петель, ожидание перед препятствием, путь в /plan.
class Drive {
public:
  struct Point {
    double x;
    double y;
  };
  enum class Result { None, Succeeded, Failed };

  explicit Drive(rclcpp::Node & node);

  // Ехать от позы робота (x, y) через points (фрейм map). Если уже едем, новый маршрут заменяет
  // старый без остановки. false — прежний запрос compass_core ещё не принял: повторить позже.
  bool follow(double x, double y, const std::vector<Point> & points);
  void stop();
  bool active() const { return active_; }

  // Чем закончилась последняя цель; один раз, потом None. Отменённая (СТОП, смена режима) — None:
  // агент поставит её заново, когда снова можно ехать.
  Result take_result() { return std::exchange(result_, Result::None); }

private:
  using NavStart = robot_interfaces::srv::NavStart;
  using NavStatus = robot_interfaces::msg::NavStatus;

  void check_status();  // наша цель в last_status_ завершилась — итог в result_
  void cancel_fms();

  rclcpp::Node & node_;
  rclcpp::Client<NavStart>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;
  rclcpp::Subscription<NavStatus>::SharedPtr status_sub_;
  NavStatus last_status_;
  std::string goal_id_;               // наша цель у compass_core; пусто — ещё не принята
  std::uint64_t request_number_ = 0;  // ответ на запрос, после которого был stop(), — отменить цель
  bool pending_ = false;              // запрос отправлен, ответа ещё нет
  bool active_ = false;
  Result result_ = Result::None;
};
