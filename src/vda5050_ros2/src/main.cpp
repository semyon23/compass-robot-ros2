#include <rclcpp/rclcpp.hpp>

#include "vda5050_ros2/adapter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdapterNode>());
  rclcpp::shutdown();
  return 0;
}
