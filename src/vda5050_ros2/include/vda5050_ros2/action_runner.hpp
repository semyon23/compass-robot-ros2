#pragma once

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <vda5050.hpp>

// Один выполняемый action. Нода опрашивает его каждый тик через update().
class ActionHandler {
public:
  virtual ~ActionHandler() = default;
  // Продвинуться на dt секунд: RUNNING, FINISHED или FAILED.
  virtual vda5050::ActionStatus update(double dt) = 0;
  virtual void pause() {}
  virtual void resume() {}
  // false — отменить нельзя (cancelAllowed = false), action доработает сам.
  virtual bool cancel() { return true; }
};

using ActionFactory = std::function<std::unique_ptr<ActionHandler>(const vda5050::Action & action)>;

// Action'ы order'а (6.2): статусы в state.actionStates, очередь по blockingType, пауза, отмена.
// Без ROS.
class ActionRunner {
public:
  explicit ActionRunner(vda5050::State & state);

  // Тип action'а: описание для factsheet (где разрешён, blockingType'ы) и как его выполнять.
  void add_type(vda5050::MobileRobotAction description, ActionFactory factory);
  const std::vector<vda5050::MobileRobotAction> & descriptions() const { return descriptions_; }
  const vda5050::MobileRobotAction * find(const std::string & type) const;

  void clear();                                                // новый order: старые action'ы забыты
  void add_waiting(const std::vector<vda5050::Action> & actions);  // в actionStates со статусом WAITING
  void enqueue(const std::vector<vda5050::Action> & actions);  // пора выполнять: робот на узле / въехал на ребро
  void leave_edge(const std::vector<vda5050::Action> & actions);  // съехали с ребра (6.2)
  void cancel_order();                                          // cancelOrder

  // Каждый тик. standing — робот стоит: только тогда стартуют SOFT и HARD.
  void update(double dt, bool standing, bool paused);

  bool blocks_driving() const;  // в очереди SOFT или HARD: робот должен стоять
  bool busy() const;            // в очереди есть незавершённые action'ы

private:
  struct Entry {
    vda5050::Action action;
    std::unique_ptr<ActionHandler> handler;  // пусто — ещё не стартовал
  };

  void set_status(const std::string & action_id, vda5050::ActionStatus status);

  vda5050::State & state_;
  std::vector<vda5050::MobileRobotAction> descriptions_;
  std::map<std::string, ActionFactory> factories_;
  std::vector<Entry> queue_;        // в порядке поступления (6.2.2)
  std::set<std::string> enqueued_;  // actionId, уже поставленные в очередь в этом order'е
  bool paused_ = false;
};
