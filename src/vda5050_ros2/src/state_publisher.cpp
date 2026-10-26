#include "vda5050_ros2/state_publisher.hpp"

namespace
{
constexpr auto kMaxStateInterval = std::chrono::seconds(30);  // 6.6; то же в factsheet
}

StatePublisher::StatePublisher(MqttLink & link, std::string topic_prefix, std::string manufacturer,
                               std::string serial_number)
: link_(link),
  prefix_(std::move(topic_prefix)),
  manufacturer_(std::move(manufacturer)),
  serial_number_(std::move(serial_number))
{
}

vda5050::Header StatePublisher::header(std::uint32_t header_id) const
{
  return {header_id, vda5050::make_timestamp(), vda5050::kProtocolVersion, manufacturer_, serial_number_};
}

std::string StatePublisher::connection_json(vda5050::ConnectionState connection_state)
{
  vda5050::Connection c;
  c.header = header(connection_id_++);  // headerId connection тесты не проверяют: will публикует брокер
  c.connection_state = connection_state;
  return nlohmann::json(c).dump();
}

void StatePublisher::publish_connection(vda5050::ConnectionState connection_state)
{
  link_.publish(prefix_ + "connection", connection_json(connection_state), 1, true);  // 6.5: QoS 1, retained
}

void StatePublisher::publish_factsheet(vda5050::Factsheet factsheet)
{
  factsheet.header = header(factsheet_id_);
  if (link_.publish(prefix_ + "factsheet", nlohmann::json(factsheet).dump(), 0, true)) {  // 6.10: retained
    ++factsheet_id_;
  }
}

void StatePublisher::update_state(vda5050::State & state, bool force)
{
  if (!link_.connected()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  // Позиция в eq_key не входит (меняется постоянно), но потеря локализации и смена карты — события.
  const auto & pos = *state.mobile_robot_position;
  const auto & sent = *last_sent_.mobile_robot_position;
  const bool event = !state_sent_ || state != last_sent_ || pos.localized != sent.localized ||
    pos.map_id != sent.map_id;
  if (!force && !event && now - last_sent_time_ < kMaxStateInterval) {
    return;
  }
  // headerId растёт только на отправленных: иначе fms увидит пропуск.
  state.header = header(state_id_);
  if (link_.publish(prefix_ + "state", nlohmann::json(state).dump(), 0, false)) {
    ++state_id_;
    last_sent_ = state;
    last_sent_time_ = now;
    state_sent_ = true;
  }
}

void StatePublisher::publish_visualization(const vda5050::State & state)
{
  if (!link_.connected()) {
    return;
  }
  vda5050::Visualization v;
  v.header = header(visualization_id_);
  v.reference_state_header_id = state.header.header_id;
  v.mobile_robot_position = state.mobile_robot_position;
  v.velocity = state.velocity;
  if (link_.publish(prefix_ + "visualization", nlohmann::json(v).dump(), 0, false)) {
    ++visualization_id_;
  }
}
