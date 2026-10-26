#include "vda5050_ros2/order_manager.hpp"

#include <algorithm>
#include <cmath>

namespace v = vda5050;

namespace
{
constexpr double kDefaultDeviation = 0.5;  // [м] допуск узла, если в order'е его нет

bool is_rejection(const v::Error & e)
{
  return kOrderRejectionErrors.count(e.error_type) > 0;
}

// Робот должен стоять, пока action работает (6.2).
bool standing_only(const std::vector<v::Action> & actions)
{
  return std::any_of(actions.begin(), actions.end(), [](const v::Action & a) {
    return a.blocking_type == v::BlockingType::Soft || a.blocking_type == v::BlockingType::Hard;
  });
}

bool same_content(const v::Order & a, const v::Order & b)
{
  return nlohmann::json(a.nodes) == nlohmann::json(b.nodes) && nlohmann::json(a.edges) == nlohmann::json(b.edges);
}
}  // namespace

OrderManager::OrderManager(vda5050::State & state, ActionRunner & actions)
: state_(state),
  actions_(actions)
{
}

void OrderManager::receive(const std::string & payload)
{
  v::Order order;
  try {
    // Битый JSON, нет обязательного поля, неизвестное значение enum — всё здесь.
    order = nlohmann::json::parse(payload).get<v::Order>();
  } catch (const std::exception & e) {
    return reject({v::error_type::kValidationFailure, e.what(), ""}, "", 0);
  }

  // Тот же order повторно (fms не дождался state) — молча игнорируем (6.1.4).
  if (order.order_id == order_.order_id && order.order_update_id == order_.order_update_id &&
      same_content(order, last_received_))
  {
    return;
  }
  if (const auto rejection = check(order)) {
    return reject(*rejection, order.order_id, order.order_update_id);
  }
  accept(order);
}

std::optional<OrderManager::Rejection> OrderManager::check(const v::Order & order) const
{
  const auto & nodes = order.nodes;
  const auto & edges = order.edges;

  // Action разрешён здесь (scope) и с этим blockingType — по factsheet (ActionRunner).
  auto check_action = [this](const v::Action & a, v::ActionScope scope) -> std::optional<Rejection> {
    const auto * d = actions_.find(a.action_type);
    const bool ok = d && std::count(d->action_scopes.begin(), d->action_scopes.end(), scope) &&
      (!d->blocking_types || std::count(d->blocking_types->begin(), d->blocking_types->end(), a.blocking_type));
    if (!ok) {
      return Rejection{v::error_type::kInvalidOrderAction,
                       a.action_type + " is not allowed here with this blockingType", a.action_id};
    }
    return std::nullopt;
  };

  // --- Структура
  if (nodes.empty()) {
    return Rejection{v::error_type::kValidationFailure, "order has no nodes"};
  }
  if (edges.size() != nodes.size() - 1) {
    return Rejection{v::error_type::kValidationFailure, "number of edges must be number of nodes - 1"};
  }
  const std::uint32_t first = nodes[0].sequence_id;
  if (first % 2 != 0) {
    return Rejection{v::error_type::kValidationFailure, "node sequenceId must be even"};
  }
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].sequence_id != first + 2 * i || (i > 0 && edges[i - 1].sequence_id != first + 2 * i - 1)) {
      return Rejection{v::error_type::kValidationFailure, "sequenceIds must go node, edge, node... without gaps"};
    }
    // base — сплошное начало order'а: released-узел не может идти после horizon'а,
    // ребро released, только если released узел, в который оно ведёт.
    if (nodes[i].released && (i > 0 && !nodes[i - 1].released)) {
      return Rejection{v::error_type::kValidationFailure, "released node after horizon"};
    }
    if (i > 0 && edges[i - 1].released != nodes[i].released) {
      return Rejection{v::error_type::kValidationFailure, "edge must be released together with its end node"};
    }
    if (!nodes[i].node_position) {
      return Rejection{v::error_type::kValidationFailure, "nodePosition is required"};
    }
    for (const auto & a : nodes[i].actions) {
      if (auto r = check_action(a, v::ActionScope::Node)) {
        return r;
      }
    }
    for (const auto & a : i > 0 ? edges[i - 1].actions : std::vector<v::Action>{}) {
      if (auto r = check_action(a, v::ActionScope::Edge)) {
        return r;
      }
    }
  }
  if (!nodes[0].released) {
    return Rejection{v::error_type::kValidationFailure, "first node must be released"};
  }

  // --- Робот
  if (state_.operating_mode != v::OperatingMode::Automatic &&
      state_.operating_mode != v::OperatingMode::Semiautomatic)
  {
    return Rejection{v::error_type::kMobileRobotNotAvailable, "robot is not in AUTOMATIC mode"};
  }
  for (const auto & node : nodes) {
    const auto & maps = *state_.maps;
    const bool known = std::any_of(maps.begin(), maps.end(), [&](const v::Map & m) {
      return m.map_id == node.node_position->map_id && m.map_status == v::EnableStatus::Enabled;
    });
    if (!known) {
      return Rejection{v::error_type::kUnknownMapId, "map " + node.node_position->map_id + " is not enabled"};
    }
  }

  // --- Новый order
  if (order.order_id != order_.order_id) {
    if (!v::is_idle(state_)) {
      return Rejection{v::error_type::kOtherOrderActive, "robot is executing order " + state_.order_id};
    }
    if (order.order_update_id != 0) {
      return Rejection{v::error_type::kValidationFailure, "new order must have orderUpdateId 0"};
    }
    // Первый узел робот считает пройденным (6.1.1): робот должен стоять в его допуске.
    const auto & pos = *state_.mobile_robot_position;
    const auto & start = *nodes[0].node_position;
    const double allowed = start.allowed_deviation_xy && start.allowed_deviation_xy->a > 0
      ? start.allowed_deviation_xy->a : kDefaultDeviation;
    if (!pos.localized) {
      return Rejection{v::error_type::kMobileRobotNotAvailable, "robot is not localized"};
    }
    if (std::hypot(pos.x - start.x, pos.y - start.y) > allowed) {
      return Rejection{v::error_type::kStartNodeOutOfRange, "robot is not at the first node"};
    }
    return std::nullopt;
  }

  // --- Update того же order'а
  if (canceled_) {
    return Rejection{v::error_type::kOrderUpdateFollowingCancel, "order was cancelled"};
  }
  if (order.order_update_id < order_.order_update_id) {
    return Rejection{v::error_type::kOutdatedOrderUpdate, "orderUpdateId is older than the current one"};
  }
  if (order.order_update_id == order_.order_update_id) {
    return Rejection{v::error_type::kSameOrderUpdateId, "same orderUpdateId with different content"};
  }
  // Update начинается с decision point — последнего released-узла текущего order'а (рисунок 8).
  const auto last_base = std::find_if(order_.nodes.rbegin(), order_.nodes.rend(),
                                      [](const v::Node & n) { return n.released; });
  if (last_base == order_.nodes.rend() || nodes[0].node_id != last_base->node_id ||
      nodes[0].sequence_id != last_base->sequence_id)
  {
    return Rejection{v::error_type::kValidationFailure, "update must start at the decision point"};
  }
  return std::nullopt;
}

void OrderManager::accept(const v::Order & order)
{
  // Ошибки отказа, NODE_UNREACHABLE прошлого order'а и предупреждения instant action'ов
  // снимает следующий принятый order (6.1.4).
  std::erase_if(state_.errors, [](const v::Error & e) {
    return is_rejection(e) || e.error_type == v::error_type::kNodeUnreachable ||
           e.error_type == v::error_type::kInvalidInstantAction || e.error_type == v::error_type::kNoOrderToCancel;
  });

  if (order.order_id != order_.order_id) {
    order_ = order;
    canceled_ = false;
    state_.order_id = order.order_id;
    actions_.clear();
    state_.last_node_id = order.nodes[0].node_id;
    state_.last_node_sequence_id = order.nodes[0].sequence_id;
  } else {
    // Склейка: старый horizon (и узел склейки) заменяются update'ом.
    const std::uint32_t stitch = order.nodes[0].sequence_id;
    std::erase_if(order_.nodes, [stitch](const v::Node & n) { return n.sequence_id >= stitch; });
    std::erase_if(order_.edges, [stitch](const v::Edge & e) { return e.sequence_id > stitch; });
    order_.nodes.insert(order_.nodes.end(), order.nodes.begin(), order.nodes.end());
    order_.edges.insert(order_.edges.end(), order.edges.begin(), order.edges.end());
    order_.order_update_id = order.order_update_id;
  }
  // Все action'ы order'а — в actionStates со статусом WAITING (6.6.5).
  for (const auto & n : order.nodes) {
    actions_.add_waiting(n.actions);
  }
  for (const auto & e : order.edges) {
    actions_.add_waiting(e.actions);
  }
  state_.order_update_id = order.order_update_id;
  last_received_ = order;
  update_states();
  // Action'ы узла, где робот стоит, и ребра из него: первый узел order'а и ребро, открытое update'ом.
  enter_current_node();
}

void OrderManager::reject(const Rejection & rejection, const std::string & order_id, std::uint32_t update_id)
{
  v::Error error;
  error.error_type = rejection.type;
  error.error_level = v::ErrorLevel::Warning;
  error.error_description = rejection.description;
  std::vector<v::ErrorReference> refs;
  if (!order_id.empty()) {
    refs = {{"orderId", order_id}, {"orderUpdateId", std::to_string(update_id)}};
  }
  if (!rejection.action_id.empty()) {
    refs.push_back({"actionId", rejection.action_id});
  }
  if (!refs.empty()) {
    error.error_references = refs;
  }
  // Висит до следующего принятого order'а; предыдущий отказ заменяется новым.
  std::erase_if(state_.errors, is_rejection);
  state_.errors.push_back(std::move(error));
}

bool OrderManager::cancel()
{
  if (v::is_idle(state_)) {
    return false;
  }
  const std::uint32_t last = state_.last_node_sequence_id;
  std::erase_if(order_.nodes, [last](const v::Node & n) { return n.sequence_id > last; });
  std::erase_if(order_.edges, [last](const v::Edge & e) { return e.sequence_id > last; });
  canceled_ = true;
  actions_.cancel_order();
  std::erase_if(state_.errors, [](const v::Error & e) { return e.error_type == v::error_type::kNodeUnreachable; });
  update_states();
  return true;
}

std::vector<v::Node> OrderManager::base_ahead() const
{
  std::vector<v::Node> ahead;
  for (const auto & node : order_.nodes) {
    if (node.sequence_id > state_.last_node_sequence_id && node.released) {
      ahead.push_back(node);
    }
  }
  return ahead;
}

std::vector<v::Node> OrderManager::base_route() const
{
  std::vector<v::Node> route;
  for (const auto & node : order_.nodes) {
    if (node.sequence_id >= state_.last_node_sequence_id && node.released) {
      route.push_back(node);
    }
  }
  return route;
}

std::vector<v::Node> OrderManager::segment() const
{
  std::vector<v::Node> nodes;
  for (const auto & node : base_ahead()) {
    nodes.push_back(node);
    const auto out = std::find_if(order_.edges.begin(), order_.edges.end(), [&node](const v::Edge & e) {
      return e.sequence_id == node.sequence_id + 1;
    });
    // SOFT/HARD узла или ребра из него выполняются стоя на этом узле.
    if (standing_only(node.actions) || (out != order_.edges.end() && out->released && standing_only(out->actions))) {
      break;
    }
  }
  return nodes;
}

void OrderManager::node_passed(const v::Node & node)
{
  // Съехали с ребра, которое вело к узлу (6.2).
  for (const auto & e : order_.edges) {
    if (e.sequence_id == node.sequence_id - 1) {
      actions_.leave_edge(e.actions);
    }
  }
  state_.last_node_id = node.node_id;
  state_.last_node_sequence_id = node.sequence_id;
  // Пройденное больше не нужно; сам узел остаётся — он может быть decision point.
  std::erase_if(order_.nodes, [&node](const v::Node & n) { return n.sequence_id < node.sequence_id; });
  std::erase_if(order_.edges, [&node](const v::Edge & e) { return e.sequence_id < node.sequence_id; });
  update_states();
  enter_current_node();
}

void OrderManager::enter_current_node()
{
  const std::uint32_t last = state_.last_node_sequence_id;
  for (const auto & n : order_.nodes) {
    if (n.sequence_id == last) {
      actions_.enqueue(n.actions);
    }
  }
  for (const auto & e : order_.edges) {
    if (e.sequence_id == last + 1 && e.released) {
      actions_.enqueue(e.actions);  // ActionRunner сам пропускает уже поставленные
    }
  }
}

void OrderManager::update_states()
{
  const std::uint32_t last = state_.last_node_sequence_id;
  state_.node_states.clear();
  for (const auto & n : order_.nodes) {
    if (n.sequence_id > last) {
      state_.node_states.push_back({n.node_id, n.sequence_id, n.node_descriptor, n.released, n.node_position});
    }
  }
  state_.edge_states.clear();
  for (const auto & e : order_.edges) {
    if (e.sequence_id > last) {
      state_.edge_states.push_back({e.edge_id, e.sequence_id, e.edge_descriptor, e.released, e.trajectory});
    }
  }
  // Released впереди нет, а horizon есть — робот на decision point, ждёт продления base.
  const bool base_left = std::any_of(state_.node_states.begin(), state_.node_states.end(),
                                     [](const v::NodeState & n) { return n.released; });
  state_.new_base_request = !base_left && !state_.node_states.empty();
}
