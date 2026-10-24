#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace compass_core {

struct MapEntry {
  std::string id, name;
  uint32_t width = 0, height = 0;  // клетки
  double resolution = 0.0;         // м/клетка
  double origin_x = 0.0, origin_y = 0.0, origin_theta = 0.0;
  bool has_posegraph = false;
  uint64_t size_bytes = 0;
  int64_t created_at = 0, updated_at = 0;  // unix, с
};

// Библиотека карт на диске (ROS_INTERFACES.md §5):
//   {root}/maps/{id}/map.pgm, map.yaml (как у map_saver), map.posegraph, map.data, meta.yaml
//   {root}/uploads/{upload_id}/…   — файлы для импорта, их кладёт compass_web (PUT)
//   {root}/active_map              — id активной карты (переживает перезапуск)
// Ошибки — std::runtime_error с причиной для человека.
class MapLibrary {
public:
  explicit MapLibrary(const std::filesystem::path & root);

  std::vector<MapEntry> list() const;  // новые сверху
  std::optional<MapEntry> get(const std::string & id) const;
  std::optional<MapEntry> find_by_name(const std::string & name) const;
  // Каталог карты; сюда же slam_toolbox пишет map.posegraph и map.data.
  std::filesystem::path dir(const std::string & id) const;

  // Записать карту из OccupancyGrid: id пустой — новая карта, иначе перезаписать эту.
  MapEntry save(const std::string & id, const std::string & name, const nav_msgs::msg::OccupancyGrid & grid);
  MapEntry rename(const std::string & id, const std::string & name);
  void remove(const std::string & id);
  MapEntry import(const std::string & upload_id, const std::string & name);

  std::string active_id() const;              // пусто — активной карты нет
  void set_active_id(const std::string & id);  // пусто — сбросить

private:
  MapEntry read(const std::string & id) const;
  void write_meta(const MapEntry & e) const;

  std::filesystem::path maps_, uploads_, active_file_;
};

}  // namespace compass_core
