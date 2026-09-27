#ifndef LWR_HW____LWR_HW_SIM_H
#define LWR_HW____LWR_HW_SIM_H

// ROS
#include <angles/angles.h>

// Gazebo hook
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros2_control/gazebo_system_interface.hpp>

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

namespace lwr_hw {

/**
 * Gazebo (classic) outlet of the LWR 4+, loaded by the gazebo_ros2_control plugin with
 * <plugin>lwr_hw/LWRHWGazebo</plugin> in the <hardware> tag of the ros2_control URDF description.
 */
class LWRHWGazebo : public gazebo_ros2_control::GazeboSystemInterface, public LWRHW
{
public:

  LWRHWGazebo() : LWRHW(), ts_(rclcpp::Duration::from_seconds(3.0)) {}
  ~LWRHWGazebo() {}

  // Gazebo hook
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

  // ros2_control
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // read and write, with Gazebo hooks
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // Gazebo stuff
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::ModelPtr parent_model_;
  bool parent_set_ = false;
  rclcpp::Duration ts_;
  rclcpp::Time lastT_{0, 0, RCL_ROS_TIME};

};

}

#endif
