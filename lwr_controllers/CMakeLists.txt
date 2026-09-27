cmake_minimum_required(VERSION 3.8)
project(lwr_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(angles REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(lwr_hw REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  angles
  control_toolbox
  controller_interface
  geometry_msgs
  hardware_interface
  kdl_parser
  lwr_hw
  orocos_kdl
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PoseRPY.msg
  msg/RPY.msg
  msg/MultiPriorityTask.msg
  msg/ArmState.msg
  msg/CartesianImpedancePoint.msg
  msg/Stiffness.msg
  srv/SetCartesianImpedanceCommand.srv
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# the controllers plugin library (the ${PROJECT_NAME} target name is taken by the interfaces)
add_library(${PROJECT_NAME}_plugin SHARED
  src/joint_impedance_controller.cpp
  src/inverse_dynamics_controller.cpp
  src/computed_torque_controller.cpp
  src/one_task_inverse_kinematics.cpp
  src/multi_task_priority_inverse_kinematics.cpp
  src/multi_task_priority_inverse_dynamics.cpp
  src/minimum_effort_inverse_dynamics.cpp
  src/backstepping_controller.cpp
  src/dynamic_sliding_mode_controller.cpp
  src/dynamic_sliding_mode_controller_task_space.cpp
  src/one_task_inverse_dynamics_jl.cpp
  src/gravity_compensation.cpp
  src/arm_state_controller.cpp
  src/cartesian_impedance_controller.cpp
  src/KinematicChainControllerBase.cpp
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_plugin PUBLIC "${cpp_typesupport_target}" Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_plugin PUBLIC ${THIS_PACKAGE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface lwr_controllers_plugins.xml)

# helper node to command the two arms of the vito robot with a single pose
add_executable(cartesian_command_vito src/cartesian_command_vito.cpp)
target_link_libraries(cartesian_command_vito "${cpp_typesupport_target}" Eigen3::Eigen)
ament_target_dependencies(cartesian_command_vito rclcpp orocos_kdl tf2 tf2_geometry_msgs tf2_ros)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS cartesian_command_vito
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS} rosidl_default_runtime)
ament_package()
