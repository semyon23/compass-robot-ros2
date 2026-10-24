#include "compass_core/config_store.hpp"

#include <chrono>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace compass_core {

namespace {

using json = nlohmann::json;

// YAML-узел → JSON: схема пишется на YAML (удобно править), UI получает JSON.
json from_yaml(const YAML::Node & n)
{
  if (n.IsMap()) {
    json j = json::object();
    for (const auto & kv : n) {
      j[kv.first.as<std::string>()] = from_yaml(kv.second);
    }
    return j;
  }
  if (n.IsSequence()) {
    json j = json::array();
    for (const auto & item : n) {
      j.push_back(from_yaml(item));
    }
    return j;
  }
  if (!n.IsScalar()) {
    return nullptr;
  }
  // Скаляр: числа и true/false — как числа и bool, остальное строкой ("" и «5» в кавычках — тоже).
  const std::string s = n.Scalar();
  if (n.Tag() != "!") {
    if (s == "true" || s == "false") {
      return s == "true";
    }
    try {
      std::size_t pos = 0;
      const long long i = std::stoll(s, &pos);
      if (pos == s.size()) {
        return i;
      }
      const double d = std::stod(s, &pos);
      if (pos == s.size()) {
        return d;
      }
    } catch (const std::exception &) {
    }
  }
  return s;
}

int64_t unix_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

json error(const char * code, const std::string & message, json extra = json::object())
{
  extra["code"] = code;
  extra["message"] = message;
  return extra;
}

}  // namespace

ConfigStore::ConfigStore(const fs::path & dir, const fs::path & schema_file, const json & initial) : dir_(dir)
{
  schema_ = from_yaml(YAML::LoadFile(schema_file.string()));
  fs::create_directories(dir_);

  // Значения по умолчанию из схемы, поверх — initial, поверх — robot.yaml, если он есть.
  for (const auto & section : schema_["sections"]) {
    for (const auto & f : section["fields"]) {
      values_[f["key"].get<std::string>()] = f["default"];
    }
  }
  for (const auto & [key, value] : initial.items()) {
    if (values_.contains(key)) {
      values_[key] = value;
    }
  }
  const fs::path file = dir_ / "robot.yaml";
  if (fs::exists(file)) {
    const json saved = from_yaml(YAML::LoadFile(file.string()));
    version_ = saved.value("version", 0);
    for (const auto & [section, fields] : saved.items()) {
      if (!fields.is_object()) {
        continue;
      }
      for (const auto & [name, value] : fields.items()) {
        const std::string key = section + "." + name;
        if (field(key)) {
          values_[key] = value;
        }
      }
    }
  } else {
    write(json::array(), "начальные значения", "compass_core");
  }
}

const nlohmann::json * ConfigStore::field(const std::string & key) const
{
  for (const auto & section : schema_["sections"]) {
    for (const auto & f : section["fields"]) {
      if (f["key"] == key) {
        return &f;
      }
    }
  }
  return nullptr;
}

ConfigStore::Result ConfigStore::apply(int base_version, const json & values, const std::string & comment,
                                       const std::string & client)
{
  Result r;
  if (!values.is_object()) {
    r.code = "INVALID_INPUT";
    r.message = "values — объект { ключ: значение }";
    return r;
  }
  if (base_version != version_) {
    r.code = "CONFLICT";
    r.message = "настройки уже изменили — обновите страницу";
    return r;
  }
  json merged = values_;
  json keys = json::array();
  for (const auto & [key, value] : values.items()) {
    if (!field(key)) {
      r.errors[key] = error("UNKNOWN_KEY", "нет такой настройки");
      continue;
    }
    if (merged[key] != value) {
      merged[key] = value;
      keys.push_back(key);
    }
  }
  r.errors.update(validate(merged, keys));
  if (!r.errors.empty()) {
    r.code = "INVALID_INPUT";
    r.message = "проверьте значения";
    return r;
  }
  r.ok = true;
  if (keys.empty()) {
    return r;  // ничего не поменялось — новая версия не нужна
  }
  json changes = json::array();
  for (const auto & key : keys) {
    changes.push_back({{"key", key}, {"from", values_[key.get<std::string>()]}, {"to", merged[key.get<std::string>()]}});
  }
  values_ = merged;
  ++version_;
  write(changes, comment, client);
  r.changed = keys;
  return r;
}

// Проверка изменённых полей по схеме и связи между порогами батареи.
nlohmann::json ConfigStore::validate(const json & merged, const json & changed_keys) const
{
  json errors = json::object();
  for (const auto & k : changed_keys) {
    const std::string key = k.get<std::string>();
    const json & f = *field(key);
    const json & v = merged[key];
    const std::string type = f["type"];
    if (type == "string") {
      if (!v.is_string()) {
        errors[key] = error("NOT_A_STRING", "нужен текст");
      } else if (f.value("required", false) && v.get<std::string>().empty()) {
        errors[key] = error("REQUIRED", "обязательное поле");
      } else if (f.contains("maxLength") && v.get<std::string>().size() > f["maxLength"].get<std::size_t>()) {
        errors[key] = error("TOO_LONG", "слишком длинно", {{"max", f["maxLength"]}});
      }
    } else if (!v.is_number() || (type == "integer" && !v.is_number_integer())) {
      errors[key] = error(type == "integer" ? "NOT_INTEGER" : "NOT_A_NUMBER", "нужно число");
    } else if (v.get<double>() < f["min"].get<double>() || v.get<double>() > f["max"].get<double>()) {
      errors[key] = error("OUT_OF_RANGE", "вне допустимого диапазона", {{"min", f["min"]}, {"max", f["max"]}});
    }
  }
  // Пороги батареи: критический ≤ минимума для задач ≤ предупреждения.
  const auto pct = [&merged](const char * key) { return merged[key].is_number() ? merged[key].get<double>() : 0.0; };
  if (pct("battery.critical_percent") > pct("battery.min_task_percent")) {
    errors["battery.critical_percent"] = error("ORDER", "критический порог выше минимума для задач");
  }
  if (pct("battery.min_task_percent") > pct("battery.warn_percent")) {
    errors["battery.min_task_percent"] = error("ORDER", "минимум для задач выше порога предупреждения");
  }
  return errors;
}

ConfigStore::Result ConfigStore::rollback(int version, const std::string & client)
{
  std::ifstream in(dir_ / "history.jsonl");
  for (std::string line; std::getline(in, line);) {
    const json h = json::parse(line, nullptr, false);
    if (!h.is_discarded() && h.value("version", -1) == version) {
      json values = json::object();
      for (const auto & [key, value] : h["values"].items()) {
        if (field(key)) {
          values[key] = value;
        }
      }
      return apply(version_, values, "откат к версии " + std::to_string(version), client);
    }
  }
  Result r;
  r.code = "NOT_FOUND";
  r.message = "нет такой версии";
  return r;
}

nlohmann::json ConfigStore::history() const
{
  json out = json::array();
  std::ifstream in(dir_ / "history.jsonl");
  for (std::string line; std::getline(in, line);) {
    json h = json::parse(line, nullptr, false);
    if (h.is_discarded()) {
      continue;
    }
    h.erase("values");
    out.insert(out.begin(), h);
  }
  return out;
}

// robot.yaml — по разделам (так его удобнее читать другим компонентам и человеку); запись через
// временный файл, чтобы при отключении питания не остаться с половиной конфига.
void ConfigStore::write(const json & changes, const std::string & comment, const std::string & client)
{
  YAML::Emitter out;
  out.SetDoublePrecision(10);  // 0.35, а не 0.34999999999999998
  out << YAML::BeginMap << YAML::Key << "version" << YAML::Value << version_;
  for (const auto & section : schema_["sections"]) {
    out << YAML::Key << section["id"].get<std::string>() << YAML::Value << YAML::BeginMap;
    for (const auto & f : section["fields"]) {
      const std::string key = f["key"];
      const json & v = values_[key];
      out << YAML::Key << key.substr(key.find('.') + 1) << YAML::Value;
      if (v.is_number_integer()) {
        out << v.get<long long>();
      } else if (v.is_number()) {
        out << v.get<double>();
      } else {
        out << YAML::DoubleQuoted << v.get<std::string>();
      }
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  const fs::path tmp = dir_ / "robot.yaml.tmp";
  std::ofstream(tmp) << "# Настройки робота — правятся из Compass (Настройки → Робот), версия растёт.\n"
                     << out.c_str() << "\n";
  fs::rename(tmp, dir_ / "robot.yaml");

  const json h = {{"version", version_}, {"createdAt", unix_ms()}, {"client", client},
                  {"comment", comment}, {"changes", changes}, {"values", values_}};
  std::ofstream(dir_ / "history.jsonl", std::ios::app) << h.dump() << "\n";
}

}  // namespace compass_core
