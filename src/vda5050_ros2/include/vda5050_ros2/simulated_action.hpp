#pragma once

#include "vda5050_ros2/action_runner.hpp"

// simulatedAction: длится duration секунд (по умолчанию 3) и ничего не делает. Нужен, чтобы
// гонять тесты соответствия и fms без механизмов робота. Включается параметром simulated_action.
void add_simulated_action(ActionRunner & runner);
