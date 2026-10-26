#include "vda5050_ros2/mqtt_link.hpp"

#include <chrono>
#include <utility>

MqttLink::MqttLink(const std::string & host, int port, const std::string & client_id,
                   std::string user, std::string password, std::vector<std::string> subscriptions,
                   std::string will_topic)
: client_("tcp://" + host + ":" + std::to_string(port), client_id),  // без persistence
  user_(std::move(user)),
  password_(std::move(password)),
  subscriptions_(std::move(subscriptions)),
  will_topic_(std::move(will_topic))
{
  client_.set_connected_handler([this](const std::string &) {
    // clean session: подписки после каждого переподключения заново. order и instantActions — QoS 0 (VDA 4.1).
    for (const auto & topic : subscriptions_) {
      client_.subscribe(topic, 0);
    }
    connected_ = true;
    link_up_ = true;
  });
  client_.set_connection_lost_handler([this](const std::string &) { connected_ = false; });
  client_.set_message_callback([this](mqtt::const_message_ptr msg) {
    std::lock_guard lock(inbox_mutex_);
    inbox_.push_back({msg->get_topic(), msg->to_string()});
  });
}

MqttLink::~MqttLink()
{
  // Колбэки обращаются к this — до разрушения полей их надо отключить.
  client_.disable_callbacks();
}

void MqttLink::connect(const std::string & will_payload)
{
  auto options = mqtt::connect_options_builder()
    .clean_session(true)
    .keep_alive_interval(std::chrono::seconds(10))  // обрыв брокер заметит за ~15 с и опубликует will
    .connect_timeout(std::chrono::seconds(2))       // меньше периода повторов в ноде
    .will(mqtt::message(will_topic_, will_payload, 1, true))
    .finalize();
  if (!user_.empty()) {
    options.set_user_name(user_);
    options.set_password(password_);
  }
  try {
    client_.connect(options);
  } catch (const mqtt::exception &) {
    // Предыдущая попытка ещё идёт — следующий повтор через период таймера ноды.
  }
}

std::vector<MqttLink::Message> MqttLink::take_messages()
{
  std::lock_guard lock(inbox_mutex_);
  return std::exchange(inbox_, {});
}

bool MqttLink::publish(const std::string & topic, const std::string & payload, int qos, bool retained)
{
  if (!connected_) {
    return false;
  }
  try {
    client_.publish(topic, payload.data(), payload.size(), qos, retained);
    return true;
  } catch (const mqtt::exception &) {
    return false;  // связь оборвалась между проверкой и отправкой
  }
}

void MqttLink::disconnect(const std::string & payload)
{
  if (!connected_) {
    return;
  }
  try {
    client_.publish(will_topic_, payload.data(), payload.size(), 1, true)->wait_for(std::chrono::seconds(1));
    client_.disconnect()->wait_for(std::chrono::seconds(1));
  } catch (const mqtt::exception &) {
  }
  connected_ = false;
}
