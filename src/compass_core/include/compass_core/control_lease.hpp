#pragma once

#include <chrono>
#include <string>

namespace compass_core {

// Аренда управления: ездить джойстиком может только один клиент Compass.
// Чистая логика без ROS — время передаётся снаружи, так её проще читать и проверять.
class ControlLease {
public:
  using Clock = std::chrono::steady_clock;

  struct Acquired {
    bool granted = false;
    std::string lease_id;
    std::string previous_holder_id;  // кого вытеснили при force; пусто — никого
  };

  Acquired acquire(const std::string & client_id, const std::string & client_name, bool force,
                   Clock::time_point now);
  bool release(const std::string & lease_id);
  // Продлевает аренду, если lease_id действующий; возвращает, действующий ли он.
  bool touch(const std::string & lease_id, Clock::time_point now);
  // Снимает аренду, если от владельца нет вестей дольше timeout; true — сняла.
  bool expire(Clock::time_point now, Clock::duration timeout);
  void clear();

  bool active() const { return !lease_id_.empty(); }
  const std::string & holder_id() const { return holder_id_; }
  const std::string & holder_name() const { return holder_name_; }

private:
  std::string holder_id_;
  std::string holder_name_;
  std::string lease_id_;
  Clock::time_point last_seen_{};
};

}  // namespace compass_core
