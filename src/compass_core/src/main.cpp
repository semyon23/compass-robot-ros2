#include <rclcpp/rclcpp.hpp>

#include "compass_core/compass_core.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Однопоточный executor: в срезе 1 нет долгих вызовов, и состоянию не нужны мьютексы.
  rclcpp::spin(std::make_shared<compass_core::CompassCore>());
  rclcpp::shutdown();
  return 0;
}
