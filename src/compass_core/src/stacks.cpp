#include "compass_core/compass_core.hpp"

#include <string_view>

using namespace std::chrono_literals;

namespace compass_core {

namespace {

std::size_t hash_of(const nav_msgs::msg::OccupancyGrid & grid)
{
  return std::hash<std::string_view>{}(
    std::string_view(reinterpret_cast<const char *>(grid.data.data()), grid.data.size()));
}

constexpr auto kSlamStartTimeout = 60s;  // SLAM не дал карту — гасим
constexpr auto kNavStartTimeout = 120s;  // Nav2 не поднялся — гасим, ждём новой активации карты

}  // namespace

// ---------- Стеки ----------
//
// Одновременно работает один стек: SLAM (TEACH_IN) или навигация (map_server + AMCL + Nav2 на
// активной карте). Нужный стек выводится из состояния (wanted_stack), а check_stack раз в 200 мс
// приводит процессы в соответствие: гасит лишний, запускает нужный, ждёт готовности.

std::string CompassCore::wanted_stack() const
{
  if (mode_ == "STARTUP") {
    return "";
  }
  if (pending_mode_ == "TEACH_IN" || (mode_ == "TEACH_IN" && pending_mode_ != "SERVICE")) {
    return "slam";
  }
  if (!active_map_.id.empty() && !nav_failed_) {
    return "nav";
  }
  return "";
}

void CompassCore::check_stack()
{
  if (stack_.poll()) {
    on_stack_exit();
  }
  const std::string want = wanted_stack();
  const auto now = ChildProcess::Clock::now();
  if (stack_.running()) {
    if (stack_state_ != "stopping" && (stack_name_ != want || restart_stack_)) {
      stack_.stop();
      stack_state_ = "stopping";
      stack_since_ = now;
      RCLCPP_INFO(get_logger(), "Останавливаю стек %s", stack_name_.c_str());
      publish_status();
    } else if (stack_state_ == "starting") {
      if (stack_name_ == "slam" && map_) {
        stack_ready();
      } else if (stack_name_ == "nav") {
        check_nav_ready();
      }
      const auto limit = stack_name_ == "slam" ? kSlamStartTimeout : kNavStartTimeout;
      if (stack_state_ == "starting" && now - stack_since_ > limit) {
        emit_event("error", "STACK_TIMEOUT", "Stack " + stack_name_ + " did not start in time",
                   {{"stack", stack_name_}});
        if (stack_name_ == "nav") {
          nav_failed_ = true;
        } else {
          pending_mode_.clear();  // остаёмся в SERVICE
        }
        stack_.stop();
        stack_state_ = "stopping";
        stack_since_ = now;
      }
    }
  } else if (!want.empty()) {
    start_stack(want);
  }
  if (stack_name_ == "nav" && stack_state_ == "running") {
    bring_up_navigation();
  }

  const std::string before = state_;
  update_switching();
  if (state_ != before) {
    publish_status();
  }
}

void CompassCore::start_stack(const std::string & name)
{
  std::vector<std::string> argv = {"ros2", "launch", "compass_bringup", name + ".launch.py",
                                   std::string("use_sim_time:=") + (use_sim_time_ ? "true" : "false")};
  if (name == "slam" && continue_from_) {
    // slam_toolbox сам допишет .posegraph / .data
    argv.push_back("map_file:=" + (maps_->dir(continue_from_->id) / "map").string());
  }
  if (name == "nav") {
    argv.push_back("map:=" + (maps_->dir(active_map_.id) / "map.yaml").string());
  }
  restart_stack_ = false;
  if (!stack_.start(argv)) {
    emit_event("error", "STACK_FAILED", "Cannot start stack " + name, {{"stack", name}});
    if (name == "nav") {
      nav_failed_ = true;
    } else {
      pending_mode_.clear();
    }
    return;
  }
  stack_name_ = name;
  stack_state_ = "starting";
  stack_since_ = ChildProcess::Clock::now();
  if (name == "slam") {
    map_.reset();
    map_hash_ = saved_hash_ = 0;
    paused_ = false;
  } else {
    localization_up_ = navigation_up_ = navigation_request_ = false;
    localization_ = robot_interfaces::msg::Localization{};
    localization_.status = "none";
  }
  RCLCPP_INFO(get_logger(), "Запускаю стек %s", name.c_str());
  update_switching();
  publish_status();
}

// Стек nav запущен, когда активна его локализация (lifecycle_manager_localization/is_active).
// Спрашиваем раз в секунду, ответ приходит асинхронно.
void CompassCore::check_nav_ready()
{
  if (localization_up_) {
    return stack_ready();
  }
  const auto now = ChildProcess::Clock::now();
  if (now - nav_query_at_ < 1s || !localization_active_client_->service_is_ready()) {
    return;
  }
  nav_query_at_ = now;
  localization_active_client_->async_send_request(
    std::make_shared<Trigger::Request>(),
    [this](rclcpp::Client<Trigger>::SharedFuture f) { localization_up_ = f.get()->success; });
}

// Навигационная часть стека (nav.launch.py запускает её с autostart:=false): поднимаем, когда
// AMCL получил позу и публикует map→odom. Не вышло — повторяем через 5 с.
void CompassCore::bring_up_navigation()
{
  const auto now = ChildProcess::Clock::now();
  if (navigation_up_ || navigation_request_ || localization_.status == "none" || now - nav_query_at_ < 5s ||
      !navigation_manage_client_->service_is_ready()) {
    return;
  }
  nav_query_at_ = now;
  navigation_request_ = true;
  auto req = std::make_shared<ManageNodes::Request>();
  req->command = ManageNodes::Request::STARTUP;
  navigation_manage_client_->async_send_request(req, [this](rclcpp::Client<ManageNodes>::SharedFuture f) {
    navigation_request_ = false;
    if (stack_name_ != "nav" || stack_state_ != "running") {
      return;  // пока поднимали, стек уже остановили
    }
    navigation_up_ = f.get()->success;
    RCLCPP_INFO(get_logger(), "Навигация Nav2: %s", navigation_up_ ? "готова" : "не поднялась, повторю");
    if (navigation_up_) {
      push_nav_limits();  // стек свежий — лимиты скорости из настроек, а не из nav2.yaml
    }
    publish_status();
  });
  RCLCPP_INFO(get_logger(), "Поднимаю навигацию Nav2");
}

void CompassCore::stack_ready()
{
  stack_state_ = "running";
  RCLCPP_INFO(get_logger(), "Стек %s готов", stack_name_.c_str());
  if (stack_name_ == "slam" && pending_mode_ == "TEACH_IN") {
    pending_mode_.clear();
    update_switching();
    set_mode("TEACH_IN");
  }
  if (stack_name_ == "nav" && pending_pose_) {
    publish_initial_pose(*pending_pose_);
    pending_pose_.reset();
  }
  update_switching();
  publish_status();
}

void CompassCore::on_stack_exit()
{
  const std::string name = stack_name_;
  if (stack_state_ == "stopping") {
    stack_name_.clear();
    stack_state_.clear();
  } else {
    stack_state_ = "failed";
    emit_event("error", "STACK_FAILED", "Stack " + name + " exited unexpectedly", {{"stack", name}});
    if (name == "nav") {
      nav_failed_ = true;  // не перезапускаем по кругу: оператор выберет карту ещё раз
    }
  }
  RCLCPP_INFO(get_logger(), "Стек %s остановлен", name.c_str());

  if (name == "slam") {
    // Из TEACH_IN — в SERVICE: по запросу, после падения SLAM или по таймауту запуска.
    // Карту, сохранённую за этот сеанс (или достроенную), делаем активной — на ней и поедем.
    // Фрейм карты SLAM и сохранённой карты один — последняя поза из SLAM годится для AMCL.
    if (mode_ == "TEACH_IN" && !map_ref_.id.empty()) {
      set_active_map(map_ref_);
      nav_failed_ = false;
      pending_pose_ = slam_pose_;
    }
    slam_pose_.reset();
    pending_mode_.clear();
    map_.reset();
    map_hash_ = saved_hash_ = 0;
    paused_ = false;
    map_ref_ = robot_interfaces::msg::MapRef{};
    continue_from_.reset();
    update_switching();
    set_mode("SERVICE");
  } else {
    localization_ = robot_interfaces::msg::Localization{};
    localization_.status = "none";
    localization_up_ = navigation_up_ = navigation_request_ = false;
  }
  tf_buffer_->clear();  // иначе ещё ~10 с поза считалась бы во фрейме map остановленного стека
  update_switching();
  publish_status();
}

// switching — пока меняем режим со сменой стека или (пере)запускаем стек.
void CompassCore::update_switching()
{
  const bool busy = !pending_mode_.empty() || stack_state_ == "starting" || stack_state_ == "stopping";
  if (!busy) {
    state_ = "ready";
    transition_ = robot_interfaces::msg::Transition{};
    return;
  }
  if (!transition_.active) {
    transition_.started_at = now();
  }
  state_ = "switching";
  transition_.active = true;
  transition_.from_mode = mode_;
  transition_.to_mode = pending_mode_.empty() ? mode_ : pending_mode_;
  transition_.step = stack_state_ == "stopping" ? "stopping_stack" : "starting_stack";
  transition_.stack = stack_name_.empty() ? wanted_stack() : stack_name_;
}

// ---------- Картографирование ----------

void CompassCore::on_map(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (stack_name_ != "slam" || !stack_.running()) {
    return;  // карта от map_server навигации или от уже остановленного SLAM
  }
  const bool was_unsaved = map_hash_ != saved_hash_;
  const std::size_t h = hash_of(*msg);
  // Продолжаем сохранённую карту: первая карта от SLAM — она и есть, изменений нет.
  if (!map_ && !map_ref_.id.empty()) {
    saved_hash_ = h;
  }
  map_ = msg;
  map_hash_ = h;
  if (mode_ == "TEACH_IN" && was_unsaved != (map_hash_ != saved_hash_)) {
    publish_status();
  }
}

void CompassCore::on_save(RequestId id, SaveMap::Request::SharedPtr req)
{
  auto res = std::make_shared<SaveMap::Response>();
  auto reply = [this, id, res] { save_srv_->send_response(*id, *res); };

  std::string name = req->name;
  if (mode_ != "TEACH_IN" || state_ != "ready" || !map_) {
    fail(res, "MODE_NOT_ALLOWED", "сохранить можно только построенную карту в режиме TEACH_IN");
    return reply();
  }
  if (!clean_name(name)) {
    fail(res, "INVALID_INPUT", "имя карты — от 1 до 64 символов");
    return reply();
  }
  // Имя занято другой картой — перезаписываем её только с overwrite.
  std::string target = map_ref_.id;
  if (const auto existing = maps_->find_by_name(name)) {
    if (existing->id != map_ref_.id && !req->overwrite) {
      fail(res, "CONFLICT", "карта «" + name + "» уже есть");
      return reply();
    }
    target = existing->id;
  } else if (name != map_ref_.name) {
    target.clear();  // новое имя — новая карта, прежняя остаётся в библиотеке
  }

  MapEntry e;
  try {
    e = maps_->save(target, name, *map_);
  } catch (const std::exception & ex) {
    fail(res, "INTERNAL", ex.what());
    return reply();
  }
  map_ref_.id = e.id;
  map_ref_.name = e.name;
  saved_hash_ = map_hash_;
  publish_status();
  res->success = true;
  res->map = map_info(e);

  // posegraph нужен, чтобы потом продолжить карту. Без него карта всё равно годится.
  if (!serialize_client_->service_is_ready()) {
    emit_event("warning", "POSEGRAPH_NOT_SAVED", "Map saved without posegraph", {{"name", e.name}});
    return reply();
  }
  auto request = std::make_shared<SerializePoseGraph::Request>();
  request->filename = (maps_->dir(e.id) / "map").string();
  serialize_client_->async_send_request(
    request, [this, reply, res, e](rclcpp::Client<SerializePoseGraph>::SharedFuture f) {
      if (f.get()->result != SerializePoseGraph::Response::RESULT_SUCCESS) {
        emit_event("warning", "POSEGRAPH_NOT_SAVED", "Map saved without posegraph", {{"name", e.name}});
      }
      if (const auto fresh = maps_->get(e.id)) {
        res->map = map_info(*fresh);
      }
      emit_event("info", "MAP_SAVED", "Map saved: " + e.name, {{"map_id", e.id}, {"name", e.name}});
      reply();
    });
}

void CompassCore::on_set_paused(RequestId id, SetBool::Request::SharedPtr req)
{
  // std_srvs: кода ошибки нет — код в message (контракт §1).
  auto res = std::make_shared<SetBool::Response>();
  auto reply = [this, id, res] { pause_srv_->send_response(*id, *res); };
  if (mode_ != "TEACH_IN" || state_ != "ready") {
    res->message = "MODE_NOT_ALLOWED";
    return reply();
  }
  if (req->data == paused_) {
    res->success = true;
    return reply();
  }
  if (!pause_client_->service_is_ready()) {
    res->message = "INTERNAL";
    return reply();
  }
  // У slam_toolbox пауза — переключатель, поэтому зовём, только когда состояние меняется.
  pause_client_->async_send_request(
    std::make_shared<SlamPause::Request>(),
    [this, reply, res, want = req->data](rclcpp::Client<SlamPause>::SharedFuture) {
      paused_ = want;
      res->success = true;
      publish_status();
      reply();
    });
}

// ---------- Библиотека карт и активная карта ----------

robot_interfaces::msg::MapInfo CompassCore::map_info(const MapEntry & e) const
{
  robot_interfaces::msg::MapInfo m;
  m.id = e.id;
  m.name = e.name;
  m.width = e.width;
  m.height = e.height;
  m.resolution = static_cast<float>(e.resolution);
  m.origin.x = e.origin_x;
  m.origin.y = e.origin_y;
  m.origin.theta = e.origin_theta;
  m.has_posegraph = e.has_posegraph;
  m.size_bytes = e.size_bytes;
  m.created_at.sec = static_cast<int32_t>(e.created_at);
  m.updated_at.sec = static_cast<int32_t>(e.updated_at);
  return m;
}

void CompassCore::on_list(ListMaps::Request::SharedPtr, ListMaps::Response::SharedPtr res)
{
  for (const auto & e : maps_->list()) {
    res->maps.push_back(map_info(e));
  }
  res->success = true;
}

void CompassCore::on_rename(RenameMap::Request::SharedPtr req, RenameMap::Response::SharedPtr res)
{
  std::string name = req->name;
  if (!clean_name(name)) {
    return fail(res, "INVALID_INPUT", "имя карты — от 1 до 64 символов");
  }
  if (!maps_->get(req->id)) {
    return fail(res, "NOT_FOUND", "карта не найдена");
  }
  if (const auto other = maps_->find_by_name(name); other && other->id != req->id) {
    return fail(res, "CONFLICT", "карта «" + name + "» уже есть");
  }
  res->map = map_info(maps_->rename(req->id, name));
  res->success = true;
  if (map_ref_.id == req->id) {
    map_ref_.name = name;
  }
  if (active_map_.id == req->id) {
    active_map_.name = name;
  }
  publish_status();
}

void CompassCore::on_delete(DeleteMap::Request::SharedPtr req, DeleteMap::Response::SharedPtr res)
{
  const auto e = maps_->get(req->id);
  if (!e) {
    return fail(res, "NOT_FOUND", "карта не найдена");
  }
  if (map_ref_.id == req->id && (mode_ == "TEACH_IN" || pending_mode_ == "TEACH_IN")) {
    return fail(res, "CONFLICT", "эту карту сейчас строят");
  }
  if (active_map_.id == req->id) {
    return fail(res, "CONFLICT", "карта активна — сначала выберите другую");
  }
  maps_->remove(req->id);
  emit_event("info", "MAP_DELETED", "Map deleted: " + e->name, {{"map_id", e->id}, {"name", e->name}});
  res->success = true;
}

void CompassCore::on_import(ImportMap::Request::SharedPtr req, ImportMap::Response::SharedPtr res)
{
  std::string name = req->name;
  if (!clean_name(name)) {
    return fail(res, "INVALID_INPUT", "имя карты — от 1 до 64 символов");
  }
  if (maps_->find_by_name(name)) {
    return fail(res, "CONFLICT", "карта «" + name + "» уже есть");
  }
  try {
    res->map = map_info(maps_->import(req->upload_id, name));
  } catch (const std::exception & ex) {
    return fail(res, "INVALID_INPUT", ex.what());
  }
  emit_event("info", "MAP_IMPORTED", "Map imported: " + name, {{"map_id", res->map.id}, {"name", name}});
  res->success = true;
}

void CompassCore::on_activate(ActivateMap::Request::SharedPtr req, ActivateMap::Response::SharedPtr res)
{
  if (mode_ == "STARTUP") {
    return fail(res, "MODE_NOT_ALLOWED", "робот ещё запускается");
  }
  if (mode_ == "TEACH_IN" || pending_mode_ == "TEACH_IN") {
    return fail(res, "MODE_NOT_ALLOWED", "сначала завершите картографирование");
  }
  if (!pending_mode_.empty() || stack_state_ == "stopping") {
    return fail(res, "BUSY", "робот переключает режим");
  }
  const auto e = maps_->get(req->map_id);
  if (!e) {
    return fail(res, "NOT_FOUND", "карта не найдена");
  }
  // Та же карта и навигация уже работает — только передать позу AMCL, без перезапуска.
  const bool same = active_map_.id == e->id && stack_name_ == "nav" && stack_state_ == "running";
  if (!same) {
    cancel_goal();
    robot_interfaces::msg::MapRef ref;
    ref.id = e->id;
    ref.name = e->name;
    set_active_map(ref);
    nav_failed_ = false;
    restart_stack_ = stack_name_ == "nav" && stack_.running();
    pending_pose_.reset();
    emit_event("info", "MAP_ACTIVATED", "Active map: " + e->name, {{"map_id", e->id}, {"name", e->name}});
  }
  if (req->has_initial_pose) {
    if (same) {
      publish_initial_pose(req->initial_pose);
    } else {
      pending_pose_ = req->initial_pose;
    }
  }
  res->success = true;
  publish_status();
}

void CompassCore::set_active_map(const robot_interfaces::msg::MapRef & map)
{
  active_map_ = map;
  maps_->set_active_id(map.id);
}

}  // namespace compass_core
