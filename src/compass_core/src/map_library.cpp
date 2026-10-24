#include "compass_core/map_library.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace compass_core {

namespace {

int64_t unix_now()
{
  return std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string make_id()
{
  static std::mt19937_64 rng{std::random_device{}()};
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(rng()));
  return buf;
}

// id и upload_id приходят из UI и становятся частью пути — пускаем только [0-9a-zA-Z-].
bool valid_id(const std::string & id)
{
  return !id.empty() && id.size() <= 64 && std::all_of(id.begin(), id.end(), [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '-';
  });
}

// Клетка OccupancyGrid → пиксель PGM, как map_saver в режиме trinary:
// занято (≥ 65) — 0, свободно (≤ 25) — 254, неизвестно — 205.
uint8_t pgm_value(int8_t v)
{
  if (v < 0) {
    return 205;
  }
  if (v >= 65) {
    return 0;
  }
  return v <= 25 ? 254 : 205;
}

void write_pgm(const fs::path & path, const nav_msgs::msg::OccupancyGrid & grid)
{
  const uint32_t w = grid.info.width, h = grid.info.height;
  std::ofstream f(path, std::ios::binary);
  f << "P5\n" << w << ' ' << h << "\n255\n";
  std::vector<char> row(w);
  // В OccupancyGrid строка 0 — низ карты (y растёт вверх), в картинке — верх.
  for (uint32_t y = h; y-- > 0;) {
    for (uint32_t x = 0; x < w; ++x) {
      row[x] = static_cast<char>(pgm_value(grid.data[y * w + x]));
    }
    f.write(row.data(), w);
  }
  if (!f) {
    throw std::runtime_error("не удалось записать " + path.string());
  }
}

void write_map_yaml(const fs::path & path, const MapEntry & e)
{
  std::ofstream f(path);
  f << "image: map.pgm\n"
    << "mode: trinary\n"
    << "resolution: " << e.resolution << "\n"
    << "origin: [" << e.origin_x << ", " << e.origin_y << ", " << e.origin_theta << "]\n"
    << "negate: 0\n"
    << "occupied_thresh: 0.65\n"
    << "free_thresh: 0.196\n";  // 205 (неизвестно) → 1 - 205/255 = 0.19608: не «свободно»
}

// Заголовок PGM: «P5 ширина высота maxval», между ними пробелы и комментарии «# …».
void read_pgm_header(const fs::path & path, uint32_t & width, uint32_t & height)
{
  std::ifstream f(path, std::ios::binary);
  std::string magic;
  f >> magic;
  auto next = [&f] {
    f >> std::ws;
    while (f.peek() == '#') {
      std::string comment;
      std::getline(f, comment);
      f >> std::ws;
    }
    uint32_t v = 0;
    f >> v;
    return v;
  };
  width = next();
  height = next();
  const uint32_t maxval = next();
  if (!f || magic != "P5" || width == 0 || height == 0 || maxval == 0 || maxval > 255) {
    throw std::runtime_error("map.pgm: нужна 8-битная картинка PGM (P5)");
  }
}

}  // namespace

MapLibrary::MapLibrary(const fs::path & root)
  : maps_(root / "maps"), uploads_(root / "uploads"), active_file_(root / "active_map")
{
  fs::create_directories(maps_);
  fs::create_directories(uploads_);
}

fs::path MapLibrary::dir(const std::string & id) const
{
  return maps_ / id;
}

std::vector<MapEntry> MapLibrary::list() const
{
  std::vector<MapEntry> out;
  for (const auto & d : fs::directory_iterator(maps_)) {
    try {
      out.push_back(read(d.path().filename().string()));
    } catch (const std::exception &) {
      // недописанный или чужой каталог — не карта
    }
  }
  std::sort(out.begin(), out.end(), [](const MapEntry & a, const MapEntry & b) {
    return a.updated_at > b.updated_at;
  });
  return out;
}

std::optional<MapEntry> MapLibrary::get(const std::string & id) const
{
  if (!valid_id(id)) {
    return std::nullopt;
  }
  try {
    return read(id);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<MapEntry> MapLibrary::find_by_name(const std::string & name) const
{
  for (auto & e : list()) {
    if (e.name == name) {
      return e;
    }
  }
  return std::nullopt;
}

MapEntry MapLibrary::save(const std::string & id, const std::string & name,
                          const nav_msgs::msg::OccupancyGrid & grid)
{
  MapEntry e;
  if (id.empty()) {
    e.id = make_id();
    e.created_at = unix_now();
  } else {
    e = read(id);
  }
  e.name = name;
  e.width = grid.info.width;
  e.height = grid.info.height;
  e.resolution = grid.info.resolution;
  e.origin_x = grid.info.origin.position.x;
  e.origin_y = grid.info.origin.position.y;
  const auto & q = grid.info.origin.orientation;
  e.origin_theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  e.updated_at = unix_now();

  const fs::path d = dir(e.id);
  fs::create_directories(d);
  // Старый posegraph к новой картинке не подходит; новый запишет slam_toolbox.
  fs::remove(d / "map.posegraph");
  fs::remove(d / "map.data");
  write_pgm(d / "map.pgm", grid);
  write_map_yaml(d / "map.yaml", e);
  write_meta(e);
  return read(e.id);
}

MapEntry MapLibrary::rename(const std::string & id, const std::string & name)
{
  MapEntry e = read(id);
  e.name = name;
  write_meta(e);
  return e;
}

void MapLibrary::remove(const std::string & id)
{
  if (valid_id(id)) {
    fs::remove_all(dir(id));
  }
}

MapEntry MapLibrary::import(const std::string & upload_id, const std::string & name)
{
  if (!valid_id(upload_id)) {
    throw std::runtime_error("неверный upload_id");
  }
  const fs::path src = uploads_ / upload_id;
  if (!fs::exists(src / "map.yaml") || !fs::exists(src / "map.pgm")) {
    throw std::runtime_error("нужны map.yaml и map.pgm");
  }

  MapEntry e;
  e.id = make_id();
  e.name = name;
  e.created_at = e.updated_at = unix_now();
  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile((src / "map.yaml").string());
    e.resolution = yaml["resolution"].as<double>();
    const auto origin = yaml["origin"].as<std::vector<double>>();
    if (origin.size() != 3 || e.resolution <= 0.0) {
      throw std::runtime_error("");
    }
    e.origin_x = origin[0];
    e.origin_y = origin[1];
    e.origin_theta = origin[2];
  } catch (const std::exception &) {
    throw std::runtime_error("map.yaml: нужны resolution и origin: [x, y, theta]");
  }
  read_pgm_header(src / "map.pgm", e.width, e.height);

  const fs::path d = dir(e.id);
  fs::create_directories(d);
  fs::copy_file(src / "map.pgm", d / "map.pgm");
  if (fs::exists(src / "map.posegraph") && fs::exists(src / "map.data")) {
    fs::copy_file(src / "map.posegraph", d / "map.posegraph");
    fs::copy_file(src / "map.data", d / "map.data");
  }
  // mode, negate и пороги оставляем как были, картинку называем по-нашему.
  yaml["image"] = "map.pgm";
  std::ofstream(d / "map.yaml") << yaml << "\n";
  write_meta(e);

  // Загрузку больше не держим; заодно чистим брошенные (старше часа).
  fs::remove_all(src);
  const auto hour_ago = fs::file_time_type::clock::now() - std::chrono::hours(1);
  for (const auto & u : fs::directory_iterator(uploads_)) {
    if (fs::last_write_time(u.path()) < hour_ago) {
      fs::remove_all(u.path());
    }
  }
  return read(e.id);
}

std::string MapLibrary::active_id() const
{
  std::string id;
  std::ifstream(active_file_) >> id;
  return id;
}

void MapLibrary::set_active_id(const std::string & id)
{
  if (id.empty()) {
    fs::remove(active_file_);
  } else {
    std::ofstream(active_file_) << id << "\n";
  }
}

MapEntry MapLibrary::read(const std::string & id) const
{
  const fs::path d = dir(id);
  const YAML::Node m = YAML::LoadFile((d / "meta.yaml").string());
  MapEntry e;
  e.id = id;
  e.name = m["name"].as<std::string>();
  e.width = m["width"].as<uint32_t>();
  e.height = m["height"].as<uint32_t>();
  e.resolution = m["resolution"].as<double>();
  const auto origin = m["origin"].as<std::vector<double>>();
  e.origin_x = origin.at(0);
  e.origin_y = origin.at(1);
  e.origin_theta = origin.at(2);
  e.created_at = m["created_at"].as<int64_t>();
  e.updated_at = m["updated_at"].as<int64_t>();
  e.has_posegraph = fs::exists(d / "map.posegraph") && fs::exists(d / "map.data");
  for (const auto & f : fs::directory_iterator(d)) {
    if (f.is_regular_file()) {
      e.size_bytes += f.file_size();
    }
  }
  return e;
}

// meta.yaml — то, чего нет в map.yaml: имя и время. Размер и origin дублируем, чтобы список
// карт не разбирал PGM.
void MapLibrary::write_meta(const MapEntry & e) const
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "name" << YAML::Value << e.name;
  out << YAML::Key << "width" << YAML::Value << e.width;
  out << YAML::Key << "height" << YAML::Value << e.height;
  out << YAML::Key << "resolution" << YAML::Value << e.resolution;
  out << YAML::Key << "origin" << YAML::Value << YAML::Flow
      << std::vector<double>{e.origin_x, e.origin_y, e.origin_theta};
  out << YAML::Key << "created_at" << YAML::Value << e.created_at;
  out << YAML::Key << "updated_at" << YAML::Value << e.updated_at;
  out << YAML::EndMap;
  std::ofstream(dir(e.id) / "meta.yaml") << out.c_str() << "\n";
}

}  // namespace compass_core
