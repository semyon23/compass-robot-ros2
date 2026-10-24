#include "compass_core/child_process.hpp"

#include <csignal>
#include <thread>

#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace compass_core {

ChildProcess::~ChildProcess()
{
  if (!running()) {
    return;
  }
  stop();
  while (!poll()) {
    std::this_thread::sleep_for(50ms);
  }
}

bool ChildProcess::start(const std::vector<std::string> & argv)
{
  if (running() || argv.empty()) {
    return false;
  }
  // Массив для execvp готовим до fork: после fork в многопоточном процессе можно звать
  // только async-signal-safe функции.
  std::vector<char *> args;
  for (const auto & a : argv) {
    args.push_back(const_cast<char *>(a.c_str()));
  }
  args.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    return false;
  }
  if (pid == 0) {
    setsid();                         // своя группа: Ctrl+C в терминале её не задевает, гасим сами
    prctl(PR_SET_PDEATHSIG, SIGINT);  // compass_core убили — launch всё равно остановится
    execvp(args[0], args.data());
    _exit(127);
  }
  pid_ = pid;
  stopping_ = false;
  last_signal_ = 0;
  return true;
}

void ChildProcess::stop()
{
  if (!running() || stopping_) {
    return;
  }
  stopping_ = true;
  stop_started_ = Clock::now();
  signal_group(SIGINT);
}

bool ChildProcess::poll()
{
  if (!running()) {
    return false;
  }
  int status = 0;
  if (waitpid(pid_, &status, WNOHANG) == pid_) {
    signal_group(SIGKILL);  // ноды, пережившие launch, не оставляем
    pid_ = -1;
    stopping_ = false;
    return true;
  }
  if (stopping_) {
    const auto elapsed = Clock::now() - stop_started_;
    if (elapsed > 10s && last_signal_ != SIGKILL) {
      signal_group(SIGKILL);
    } else if (elapsed > 5s && last_signal_ == SIGINT) {
      signal_group(SIGTERM);
    }
  }
  return false;
}

void ChildProcess::signal_group(int sig)
{
  kill(-pid_, sig);
  last_signal_ = sig;
}

}  // namespace compass_core
