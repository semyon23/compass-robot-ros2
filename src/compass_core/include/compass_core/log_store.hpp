#pragma once

#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

namespace compass_core {

struct LogEntry {
  int64_t ms = 0;  // unix, мс — время получения по часам робота
  uint8_t level = 0;
  std::string name, msg, file, function;
  uint32_t line = 0;
};

// Журнал WARN+ со всех нод: последние записи в памяти и файлы по дням на диске,
// чтобы история пережила перезапуск compass_core (ROS_INTERFACES.md §2.1, /rosout).
//   {dir}/compass-YYYY-MM-DD.log — строка JSON на запись, дата по UTC; старше keep_days удаляются.
// Одинаковое сообщение той же ноды чаще раза в минуту не пишется: иначе предупреждения вроде
// «AMCL: set the initial pose» каждые 2 с вытеснили бы всё остальное.
class LogStore {
public:
  LogStore(const std::filesystem::path & dir, std::size_t keep_in_memory, int keep_days);

  void add(const LogEntry & e);
  const std::deque<LogEntry> & entries() const { return entries_; }

private:
  void open_file(int64_t ms);
  void load_recent();

  std::filesystem::path dir_;
  std::size_t keep_in_memory_;
  int keep_days_;
  std::deque<LogEntry> entries_;
  std::ofstream out_;
  std::string out_date_;                                       // дата открытого файла
  std::map<std::string, std::pair<std::string, int64_t>> last_;  // нода → последнее сообщение, когда
};

}  // namespace compass_core
