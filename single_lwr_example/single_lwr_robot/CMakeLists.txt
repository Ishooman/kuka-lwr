cmake_minimum_required(VERSION 3.8)
project(single_lwr_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config robot worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
