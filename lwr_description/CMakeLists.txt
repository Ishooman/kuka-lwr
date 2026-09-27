cmake_minimum_required(VERSION 3.8)
project(lwr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model)

ament_package()
