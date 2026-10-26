#include "vda5050_ros2/simulated_action.hpp"

namespace v = vda5050;

namespace
{
class SimulatedAction : public ActionHandler {
public:
  explicit SimulatedAction(double duration)
  : left_(duration) {}

  v::ActionStatus update(double dt) override
  {
    if (!paused_) {
      left_ -= dt;
    }
    return left_ > 0 ? v::ActionStatus::Running : v::ActionStatus::Finished;
  }
  void pause() override { paused_ = true; }
  void resume() override { paused_ = false; }

private:
  double left_;
  bool paused_ = false;
};
}  // namespace

void add_simulated_action(ActionRunner & runner)
{
  v::MobileRobotAction d;
  d.action_type = "simulatedAction";
  d.action_description = "Simulated action: lasts 'duration' seconds";
  d.action_scopes = {v::ActionScope::Node, v::ActionScope::Edge};
  d.blocking_types = std::vector{v::BlockingType::None, v::BlockingType::Soft, v::BlockingType::Hard};
  d.action_parameters = std::vector<v::FactsheetActionParameter>{
    {"duration", v::ValueDataType::Number, std::string("seconds, default 3"), true}};
  d.pause_allowed = true;
  d.cancel_allowed = true;

  runner.add_type(d, [](const v::Action & action) {
    double duration = 3.0;
    for (const auto & p : action.action_parameters.value_or(std::vector<v::ActionParameter>{})) {
      if (p.key == "duration" && p.value.is_number()) {
        duration = p.value.get<double>();
      }
    }
    return std::make_unique<SimulatedAction>(duration);
  });
}
