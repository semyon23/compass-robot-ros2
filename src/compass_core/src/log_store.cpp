#include "compass_core/log_store.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <vector>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace compass_core {

namespace {

constexpr int64_t kRepeatMs = 60'000;  // то же сообщение той же ноды — не чаще раза в минуту

std::string utc_date(int64_t ms)
{
  const std::time_t t = ms / 1000;
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tm);
  return buf;
}

json to_json(const LogEntry & e)
{
  return {{"t", e.ms}, {"level", e.level}, {"name", e.name}, {"msg", e.msg},
          {"file", e.file}, {"function", e.function}, {"line", e.line}};
}

// Файлы журнала, новые первыми (имя — дата, поэтому сортируется как строка).
std::vector<fs::path> log_files(const fs::path & dir)
{
  std::vector<fs::path> files;
  for (const auto & f : fs::directory_iterator(dir)) {
    const auto name = f.path().filename().string();
    if (name.rfind("compass-", 0) == 0 && f.path().extension() == ".log") {
      files.push_back(f.path());
    }
  }
  std::sort(files.rbegin(), files.rend());
  return files;
}

}  // namespace

LogStore::LogStore(const fs::path & dir, std::size_t keep_in_memory, int keep_days)
  : dir_(dir), keep_in_memory_(keep_in_memory), keep_days_(keep_days)
{
  fs::create_directories(dir_);
  load_recent();
}

void LogStore::add(const LogEntry & e)
{
  auto & last = last_[e.name];
  if (last.first == e.msg && e.ms - last.second < kRepeatMs) {
    return;
  }
  last = {e.msg, e.ms};

  entries_.push_back(e);
  if (entries_.size() > keep_in_memory_) {
    entries_.pop_front();
  }
  open_file(e.ms);
  out_ << to_json(e).dump() << '\n' << std::flush;
}

// Новый день — новый файл; заодно удаляем старше keep_days.
void LogStore::open_file(int64_t ms)
{
  const std::string date = utc_date(ms);
  if (date == out_date_ && out_.is_open()) {
    return;
  }
  out_.close();
  out_.open(dir_ / ("compass-" + date + ".log"), std::ios::app);
  out_date_ = date;

  const auto cutoff = fs::file_time_type::clock::now() - std::chrono::hours(24 * keep_days_);
  for (const auto & f : log_files(dir_)) {
    if (fs::last_write_time(f) < cutoff) {
      fs::remove(f);
    }
  }
}

// При старте — последние keep_in_memory записей из свежих файлов.
void LogStore::load_recent()
{
  std::deque<LogEntry> loaded;
  for (const auto & path : log_files(dir_)) {
    std::vector<LogEntry> file;
    std::ifstream in(path);
    for (std::string line; std::getline(in, line);) {
      const json j = json::parse(line, nullptr, false);
      if (j.is_discarded()) {
        continue;  // недописанная строка (выключили питание)
      }
      file.push_back({j.value("t", int64_t{0}), j.value("level", uint8_t{0}), j.value("name", ""),
                      j.value("msg", ""), j.value("file", ""), j.value("function", ""), j.value("line", 0u)});
    }
    loaded.insert(loaded.begin(), file.begin(), file.end());
    if (loaded.size() >= keep_in_memory_) {
      break;
    }
  }
  while (loaded.size() > keep_in_memory_) {
    loaded.pop_front();
  }
  entries_ = std::move(loaded);
}

}  // namespace compass_core
