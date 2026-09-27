cmake_minimum_required(VERSION 3.8)
project(lwr_hw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

option(LWR_HW_BUILD_FRIL "Build the Stanford FRI library (FRIL) hardware interface (downloads https://github.com/iocroblab/fril)" ON)
option(LWR_HW_BUILD_GAZEBO "Build the Gazebo (classic) hardware interface" ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

set(THIS_PACKAGE_DEPENDS
  angles
  control_toolbox
  hardware_interface
  kdl_parser
  orocos_kdl
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  urdf
)

###########
## Build ##
###########

## LWR HW BASE CLASS
add_library(${PROJECT_NAME} SHARED
  src/lwr_hw.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_DEPENDS})

## REAL
# KUKA FRI library
add_library(friremote SHARED
  src/fri/friremote.cpp
  src/fri/friudp.cpp
)
target_include_directories(friremote PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(friremote PRIVATE HAVE_SOCKLEN_T)

# lwr hw fri plugin (replaces the ROS 1 lwr_hw_fri_node, it is loaded by the ros2_control_node)
add_library(lwr_hw_fri SHARED
  src/lwr_hw_fri.cpp
)
target_link_libraries(lwr_hw_fri PUBLIC ${PROJECT_NAME} friremote)
ament_target_dependencies(lwr_hw_fri PUBLIC ${THIS_PACKAGE_DEPENDS})
set(LWR_HW_PLUGIN_TARGETS lwr_hw_fri)

# Standford FRILibrary
if(LWR_HW_BUILD_FRIL)
  message(STATUS "ATTENTION: This project supports the fork of the Standford library found in https://github.com/iocroblab/fril, it is downloaded and built with the package (disable it with -DLWR_HW_BUILD_FRIL=OFF)")
  include(ExternalProject)
  set(FRIL_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/fril_install)
  ExternalProject_Add(FRILibrary
    GIT_REPOSITORY https://github.com/iocroblab/fril.git
    GIT_TAG a10695fad32ba8315817c42020e2dbc66f4505dd
    BUILD_COMMAND $(MAKE)
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=${FRIL_INSTALL_DIR} -DCMAKE_POSITION_INDEPENDENT_CODE=ON .
    INSTALL_COMMAND $(MAKE) install
  )
  # lwr hw fril plugin
  add_library(lwr_hw_fril SHARED
    src/lwr_hw_fril.cpp
  )
  add_dependencies(lwr_hw_fril FRILibrary)
  target_include_directories(lwr_hw_fril PRIVATE ${FRIL_INSTALL_DIR}/include/fril)
  target_link_directories(lwr_hw_fril PRIVATE ${FRIL_INSTALL_DIR}/lib)
  target_link_libraries(lwr_hw_fril PUBLIC ${PROJECT_NAME} fril)
  ament_target_dependencies(lwr_hw_fril PUBLIC ${THIS_PACKAGE_DEPENDS})
  list(APPEND LWR_HW_PLUGIN_TARGETS lwr_hw_fril)
  install(DIRECTORY ${FRIL_INSTALL_DIR}/lib/ DESTINATION lib)
endif()

pluginlib_export_plugin_description_file(hardware_interface lwr_hw_plugins.xml)

## SIMULATION
# lwr hw sim gazebo plugin, loaded by gazebo_ros2_control
if(LWR_HW_BUILD_GAZEBO)
  find_package(gazebo_dev REQUIRED)
  find_package(gazebo_ros REQUIRED)
  find_package(gazebo_ros2_control REQUIRED)
  add_library(lwr_hw_gazebo_plugin SHARED
    src/lwr_hw_gazebo.cpp
  )
  target_link_libraries(lwr_hw_gazebo_plugin PUBLIC ${PROJECT_NAME})
  ament_target_dependencies(lwr_hw_gazebo_plugin PUBLIC ${THIS_PACKAGE_DEPENDS} gazebo_dev gazebo_ros gazebo_ros2_control)
  list(APPEND LWR_HW_PLUGIN_TARGETS lwr_hw_gazebo_plugin)
  pluginlib_export_plugin_description_file(gazebo_ros2_control lwr_hw_gazebo_plugins.xml)
endif()

## INSTALLATION
install(TARGETS ${PROJECT_NAME} friremote
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# plugins are loaded at runtime, they are not exported
install(TARGETS ${LWR_HW_PLUGIN_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY krl doc
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
