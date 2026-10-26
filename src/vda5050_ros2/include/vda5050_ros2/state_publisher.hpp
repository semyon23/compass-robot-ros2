#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <vda5050.hpp>

#include "vda5050_ros2/mqtt_link.hpp"

// Сообщения робота в fms: connection, factsheet, state, visualization.
// headerId — свой счётчик на каждый топик, +1 на каждое сообщение (6.x).
class StatePublisher {
public:
  // topic_prefix — "amr/v3/<manufacturer>/<serialNumber>/".
  StatePublisher(MqttLink & link, std::string topic_prefix, std::string manufacturer,
                 std::string serial_number);

  // JSON connection; отдельно — потому что тот же JSON с CONNECTION_BROKEN нужен для last will.
  std::string connection_json(vda5050::ConnectionState connection_state);

  void publish_connection(vda5050::ConnectionState connection_state);
  void publish_factsheet(vda5050::Factsheet factsheet);

  // state — сразу при событии (изменились поля из eq_key в state.hpp), по запросу (force)
  // и не реже раза в 30 с. Вызывается каждый тик ноды, поэтому не чаще периода тика.
  void update_state(vda5050::State & state, bool force);

  void publish_visualization(const vda5050::State & state);

private:
  vda5050::Header header(std::uint32_t header_id) const;

  MqttLink & link_;
  std::string prefix_;
  std::string manufacturer_;
  std::string serial_number_;

  std::uint32_t connection_id_ = 0;
  std::uint32_t factsheet_id_ = 0;
  std::uint32_t state_id_ = 0;
  std::uint32_t visualization_id_ = 0;

  vda5050::State last_sent_;
  std::chrono::steady_clock::time_point last_sent_time_;
  bool state_sent_ = false;
};
