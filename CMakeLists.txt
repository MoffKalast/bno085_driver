cmake_minimum_required(VERSION 3.8)
project(bno085_driver)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/bno085_node.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch param DESTINATION share/${PROJECT_NAME})

ament_package()
