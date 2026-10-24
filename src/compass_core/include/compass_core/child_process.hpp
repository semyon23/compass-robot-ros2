#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <sys/types.h>

namespace compass_core {

// Стек (`ros2 launch …`) дочерним процессом в своей группе процессов: остановка сигналом
// всей группе гасит и launch, и запущенные им ноды.
class ChildProcess {
public:
  using Clock = std::chrono::steady_clock;

  ChildProcess() = default;
  ChildProcess(const ChildProcess &) = delete;
  ChildProcess & operator=(const ChildProcess &) = delete;
  ~ChildProcess();  // ждёт завершения группы — вызывается при выходе compass_core

  bool start(const std::vector<std::string> & argv);
  // SIGINT группе. Дальше poll() через 5 с шлёт SIGTERM, через 10 с — SIGKILL.
  void stop();
  // Вызывать периодически. true — процесс только что завершился (сам или после stop()).
  bool poll();
  bool running() const { return pid_ > 0; }

private:
  void signal_group(int sig);

  pid_t pid_ = -1;
  bool stopping_ = false;
  int last_signal_ = 0;
  Clock::time_point stop_started_{};
};

}  // namespace compass_core
