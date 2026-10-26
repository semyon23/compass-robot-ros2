#pragma once

#include <optional>
#include <string>
#include <vector>

#include <vda5050.hpp>

#include "vda5050_ros2/action_runner.hpp"

// Order'а (6.1): проверка и отказы (6.1.4), новый order и update, base и horizon.
// Состояние order'а ведётся прямо в State: orderId, lastNodeId, nodeStates, edgeStates,
// newBaseRequest, errors. Позу, карты и режим берёт из того же State. Action'ы узлов и рёбер
// передаёт в ActionRunner, когда робот до них доходит. Без ROS.
class OrderManager {
public:
  OrderManager(vda5050::State & state, ActionRunner & actions);

  // Разобрать order и принять его или отказать (ошибка в state.errors).
  void receive(const std::string & payload);

  // cancelOrder: base и horizon впереди убираются. false — отменять нечего.
  bool cancel();

  // Released-узлы впереди робота по порядку: base, который осталось проехать.
  std::vector<vda5050::Node> base_ahead() const;

  // Base от узла, где робот, до конца: для карты в Compass.
  std::vector<vda5050::Node> base_route() const;

  // Отрезок, который ехать без остановки: base_ahead() до первого узла, где надо стоять
  // (SOFT/HARD action узла или ребра из него) или до конца base.
  std::vector<vda5050::Node> segment() const;

  // Робот прошёл узел из base_ahead().
  void node_passed(const vda5050::Node & node);

private:
  struct Rejection {
    std::string type;  // vda5050::error_type::*
    std::string description;
    std::string action_id{};  // отказ из-за action'а — ссылка на него
  };

  std::optional<Rejection> check(const vda5050::Order & order) const;
  void accept(const vda5050::Order & order);
  void reject(const Rejection & rejection, const std::string & order_id, std::uint32_t update_id);
  void update_states();
  void enter_current_node();

  vda5050::State & state_;
  ActionRunner & actions_;
  vda5050::Order order_;          // текущий order после склейки update'ов: впереди робота base и horizon
  vda5050::Order last_received_;  // последний принятый как пришёл — узнать повтор того же update'а
  bool canceled_ = false;         // order отменён: его update'ы отклоняются (6.1.4)
};
