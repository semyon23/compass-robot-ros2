cmake_minimum_required(VERSION 3.8)
project(compass_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fake_battery.py DESTINATION lib/${PROJECT_NAME})

ament_package()
