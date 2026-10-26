#include "vda5050_ros2/action_runner.hpp"

#include <algorithm>

namespace v = vda5050;

namespace
{
// Ни с кем параллельно (6.2.2): HARD — стоя, SINGLE — на ходу.
bool exclusive(const v::Action & a)
{
  return a.blocking_type == v::BlockingType::Hard || a.blocking_type == v::BlockingType::Single;
}

bool standing_only(const v::Action & a)
{
  return a.blocking_type == v::BlockingType::Hard || a.blocking_type == v::BlockingType::Soft;
}
}  // namespace

ActionRunner::ActionRunner(v::State & state)
: state_(state)
{
}

void ActionRunner::add_type(v::MobileRobotAction description, ActionFactory factory)
{
  factories_[description.action_type] = std::move(factory);
  descriptions_.push_back(std::move(description));
}

const v::MobileRobotAction * ActionRunner::find(const std::string & type) const
{
  const auto it = std::find_if(descriptions_.begin(), descriptions_.end(),
                               [&type](const v::MobileRobotAction & d) { return d.action_type == type; });
  return it == descriptions_.end() ? nullptr : &*it;
}

void ActionRunner::clear()
{
  // Незавершённых action'ов здесь нет: новый order принимается только в idle.
  queue_.clear();
  enqueued_.clear();
  state_.action_states.clear();
}

void ActionRunner::add_waiting(const std::vector<v::Action> & actions)
{
  for (const auto & a : actions) {
    const bool known = std::any_of(state_.action_states.begin(), state_.action_states.end(),
                                   [&a](const v::ActionState & s) { return s.action_id == a.action_id; });
    if (!known) {
      v::ActionState s;
      s.action_id = a.action_id;
      s.action_type = a.action_type;
      s.action_descriptor = a.action_descriptor;
      s.action_status = v::ActionStatus::Waiting;
      state_.action_states.push_back(std::move(s));
    }
  }
}

void ActionRunner::enqueue(const std::vector<v::Action> & actions)
{
  for (const auto & a : actions) {
    if (enqueued_.insert(a.action_id).second) {
      queue_.push_back({a, nullptr});
    }
  }
}

void ActionRunner::leave_edge(const std::vector<v::Action> & actions)
{
  // Робот съехал с ребра: работающие action'ы ребра — FINISHED, так и не начатые — FAILED.
  for (const auto & a : actions) {
    const auto it = std::find_if(queue_.begin(), queue_.end(),
                                 [&a](const Entry & e) { return e.action.action_id == a.action_id; });
    if (it == queue_.end()) {
      // Уже завершён (SOFT/HARD ребра выполняются стоя до съезда) — не трогаем; так и не начат — FAILED.
      for (auto & s : state_.action_states) {
        if (s.action_id == a.action_id && s.action_status == v::ActionStatus::Waiting) {
          s.action_status = v::ActionStatus::Failed;
        }
      }
      continue;
    }
    if (it->handler) {
      it->handler->cancel();
      set_status(a.action_id, v::ActionStatus::Finished);
    } else {
      set_status(a.action_id, v::ActionStatus::Failed);
    }
    queue_.erase(it);
  }
}

void ActionRunner::cancel_order()
{
  // Так и не начатые и отменяемые — FAILED; неотменяемые дорабатывают (cancelOrder ждёт их, busy()).
  std::erase_if(queue_, [this](Entry & e) {
    if (e.handler && !e.handler->cancel()) {
      return false;
    }
    set_status(e.action.action_id, v::ActionStatus::Failed);
    return true;
  });
  for (auto & s : state_.action_states) {
    if (s.action_status == v::ActionStatus::Waiting) {
      s.action_status = v::ActionStatus::Failed;  // не дошли до узла / ребра
    }
  }
}

void ActionRunner::update(double dt, bool standing, bool paused)
{
  if (paused != paused_) {
    paused_ = paused;
    for (auto & e : queue_) {
      if (e.handler) {
        paused ? e.handler->pause() : e.handler->resume();
        set_status(e.action.action_id, paused ? v::ActionStatus::Paused : v::ActionStatus::Running);
      }
    }
  }
  if (paused_) {
    return;
  }

  // Работающие.
  std::erase_if(queue_, [this, dt](Entry & e) {
    if (!e.handler) {
      return false;
    }
    const auto status = e.handler->update(dt);
    if (status == v::ActionStatus::Running) {
      return false;
    }
    set_status(e.action.action_id, status);  // FINISHED или FAILED
    return true;
  });

  // Запуск ждущих по порядку (6.2.2): exclusive — только когда ничего не работает, и после него
  // ничего не стартует; SOFT и HARD — только стоя; порядок не обгоняем.
  bool running = std::any_of(queue_.begin(), queue_.end(), [](const Entry & e) { return e.handler != nullptr; });
  for (auto & e : queue_) {
    if (e.handler) {
      if (exclusive(e.action)) {
        break;
      }
      continue;
    }
    if ((exclusive(e.action) && running) || (standing_only(e.action) && !standing)) {
      break;
    }
    e.handler = factories_.at(e.action.action_type)(e.action);
    set_status(e.action.action_id, v::ActionStatus::Running);
    running = true;
    if (exclusive(e.action)) {
      break;
    }
  }
}

bool ActionRunner::blocks_driving() const
{
  return std::any_of(queue_.begin(), queue_.end(), [](const Entry & e) { return standing_only(e.action); });
}

bool ActionRunner::busy() const
{
  return !queue_.empty();
}

void ActionRunner::set_status(const std::string & action_id, v::ActionStatus status)
{
  for (auto & s : state_.action_states) {
    if (s.action_id == action_id) {
      s.action_status = status;
    }
  }
}
