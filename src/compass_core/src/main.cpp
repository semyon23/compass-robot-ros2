#include <rclcpp/rclcpp.hpp>

#include "compass_core/compass_core.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<compass_core::CompassCore>());
  rclcpp::shutdown();
  return 0;
}
