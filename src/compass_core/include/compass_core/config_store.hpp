#pragma once

#include <filesystem>
#include <string>

#include <nlohmann/json.hpp>

namespace compass_core {

// Настройки робота (ROS_INTERFACES.md §3.5): один файл на всех, хозяин — compass_core.
//   {dir}/robot.yaml     — текущие значения по разделам + version; его читают и другие компоненты
//   {dir}/history.jsonl  — строка JSON на версию: кто, когда, что поменял, все значения
// Значения снаружи — плоский JSON { "motion.max_linear": 0.5, … }, ключи и проверки — из схемы.
class ConfigStore {
public:
  using json = nlohmann::json;

  struct Result {
    bool ok = false;
    std::string code;     // INVALID_INPUT | CONFLICT | NOT_FOUND
    std::string message;  // для человека
    json errors = json::object();   // INVALID_INPUT: { ключ: { code, message, … } }
    json changed = json::array();   // применённые ключи
  };

  // initial — значения, которыми заполнить robot.yaml, если его ещё нет (поверх default из схемы).
  ConfigStore(const std::filesystem::path & dir, const std::filesystem::path & schema_file, const json & initial);

  const json & schema() const { return schema_; }
  int version() const { return version_; }
  const json & values() const { return values_; }

  // values — только изменённые ключи. base_version устарел — CONFLICT.
  Result apply(int base_version, const json & values, const std::string & comment, const std::string & client);
  Result rollback(int version, const std::string & client);
  json history() const;  // новые сверху, без полных значений

private:
  json validate(const json & merged, const json & changed_keys) const;
  void write(const json & changes, const std::string & comment, const std::string & client);
  const json * field(const std::string & key) const;

  std::filesystem::path dir_;
  json schema_;
  json values_ = json::object();
  int version_ = 0;
};

}  // namespace compass_core
