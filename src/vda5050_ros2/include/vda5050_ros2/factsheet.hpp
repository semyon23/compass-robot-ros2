#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vda5050.hpp>

// Factsheet робота (6.10) из параметров factsheet.* ноды (vda5050.yaml).
// visualization_interval — с каким периодом нода реально шлёт visualization;
// order_actions — action'ы узлов и рёбер (ActionRunner::descriptions()).
vda5050::Factsheet build_factsheet(rclcpp::Node & node, double visualization_interval,
                                   const std::vector<vda5050::MobileRobotAction> & order_actions);
