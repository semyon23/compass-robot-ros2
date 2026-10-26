#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <mqtt/async_client.h>

// Связь с брокером fms (paho-mqtt-cpp).
// Колбэки paho приходят в его собственном потоке. Они только кладут сообщения в очередь
// и ставят флаги, а нода забирает всё это в своём потоке (take_messages, take_link_up) —
// так вся логика агента однопоточная.
class MqttLink {
public:
  struct Message {
    std::string topic;
    std::string payload;
  };

  MqttLink(const std::string & host, int port, const std::string & client_id,
           std::string user, std::string password, std::vector<std::string> subscriptions,
           std::string will_topic);
  ~MqttLink();

  // Начать подключение (асинхронно). will — connection = CONNECTION_BROKEN, его опубликует брокер,
  // если связь оборвётся. Пока connected() == false, нода вызывает connect() повторно.
  void connect(const std::string & will_payload);
  bool connected() const { return connected_; }

  // true один раз после каждого (пере)подключения: пора отправить connection, factsheet и state.
  bool take_link_up() { return link_up_.exchange(false); }
  std::vector<Message> take_messages();

  bool publish(const std::string & topic, const std::string & payload, int qos, bool retained);

  // Штатное отключение: сначала payload (connection = OFFLINE) в will_topic, потом disconnect —
  // при штатном disconnect брокер last will не публикует.
  void disconnect(const std::string & payload);

private:
  mqtt::async_client client_;
  std::string user_;
  std::string password_;
  std::vector<std::string> subscriptions_;
  std::string will_topic_;

  std::atomic<bool> connected_{false};
  std::atomic<bool> link_up_{false};
  std::mutex inbox_mutex_;
  std::vector<Message> inbox_;
};
