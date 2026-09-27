#ifndef LWR_HW__LWR_HW_H
#define LWR_HW__LWR_HW_H

// STD
#include <limits>
#include <memory>
#include <string>
#include <vector>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

// ROS controls
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <control_toolbox/filters.hpp>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor
#include <kdl_parser/kdl_parser.hpp>

namespace lwr_hw
{

// Non-standard interface names exported by the LWR 4+ (besides position/velocity/effort)
constexpr char HW_IF_STIFFNESS[] = "stiffness";
constexpr char HW_IF_DAMPING[] = "damping";
constexpr char HW_IF_SET_POINT[] = "set_point";

/**
 * Hardware-agnostic part of the LWR 4+ ros2_control interface.
 *
 * In ROS 1 this class was the RobotHW. In ROS 2 each outlet (real FRI/FRIL, Gazebo, etc.) derives from
 * the corresponding hardware_interface base class *and* from this class, and forwards the ros2_control
 * calls to the helpers below.
 *
 * Exported interfaces, for a robot called "lwr":
 *  - joints lwr_a1_joint ... lwr_a6_joint
 *      state:   position, velocity, effort, stiffness, damping
 *      command: position, effort, stiffness, damping, set_point
 *  - cartesian variables, prefix "lwr_cart"
 *      state/command: rot_xx, rot_yx, rot_zx, pos_x, ... pos_z (12, row-major 3x4 frame),
 *                     X_stiffness ... C_stiffness, X_damping ... C_damping, X_wrench ... C_wrench
 */
class LWRHW
{
public:

  LWRHW() {}
  virtual ~LWRHW() {}

  bool create(std::string name, std::string urdf_string);

  // reads the "name", "root_name" and "tip_name" hardware parameters and the URDF from the ros2_control info
  bool create(const hardware_interface::HardwareInfo & info);

  // Strings
  std::string robot_namespace_;
  std::string cart_prefix_;

  // Model
  std::string urdf_string_;
  urdf::Model urdf_model_;
  std::string root_name_;
  std::string tip_name_;

  // control strategies
  // JOINT_POSITION -> strategy 10 -> triggered with the joint position command interface
  // CARTESIAN_IMPEDANCE -> strategy 20 -> triggered with any of the cartesian command interfaces
  // JOINT_IMPEDANCE -> strategy 30 -> triggered with the joint effort/stiffness/damping/set_point command interfaces
// JOINT_EFFORT -> strategy 40 with special configuration
// JOINT_STIFFNESS -> strategy 50 with special configuration
// GRAVITY_COMPENSATION -> (does not exist in the real robot, achieved with low stiffness)
  enum ControlStrategy {JOINT_POSITION = 10, CARTESIAN_IMPEDANCE = 20, JOINT_IMPEDANCE = 30, JOINT_EFFORT = 40, JOINT_STIFFNESS = 50, GRAVITY_COMPENSATION = 90};

  // ros2_control helpers, to be called from the export_*/prepare/perform_command_mode_switch implementations
  std::vector<hardware_interface::StateInterface> exportStateInterfaces();
  std::vector<hardware_interface::CommandInterface> exportCommandInterfaces();
  bool prepareSwitch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) const;
  virtual void doSwitch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces);

  /**
   * @brief Function to get the control strategy based on the list of interfaces to be started/stopped
   *
   * @param start_interfaces command interfaces to be started
   * @param stop_interfaces command interfaces to be stopped
   * @param default_control_strategy value to use as a default return value
   *
   * @return the output control strategy found based on the lists
   */
  ControlStrategy getNewControlStrategy(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces, ControlStrategy default_control_strategy = JOINT_POSITION) const;

  // get/set control method
  void setControlStrategy( ControlStrategy strategy){current_strategy_ = strategy;};
  ControlStrategy getControlStrategy(){ return current_strategy_;};

  ControlStrategy current_strategy_;

  // Before write, you can use this function to enforce limits for all values
  void enforceLimits(double period);

  // Semantic zero of the commands and reset of the joint limits saturation, to be done on every mode switch
  void resetCommandsOnSwitch();

  // Copy the measured joint positions into the position commands, so nothing moves before a controller starts.
  // Call it once, after the first read.
  void initCommandsFromState();
  bool commands_initialized_ = false;

  // configuration
  int n_joints_ = 7; // safe magic number, the kuka lwr 4+ has 7 joints
  std::vector<std::string> joint_names_;
  std::vector<std::string> cart_12_names_;
  std::vector<std::string> cart_6_names_;

  // limits
  std::vector<double>
  joint_lower_limits_,
  joint_upper_limits_,
  joint_velocity_limits_,
  joint_effort_limits_ ,
  joint_lower_limits_stiffness_,
  joint_upper_limits_stiffness_,
  joint_velocity_limits_stiffness_,
  joint_lower_limits_damping_,
  joint_upper_limits_damping_,
  joint_velocity_limits_damping_;

  // state and commands
  std::vector<double>
  joint_position_,
  joint_position_prev_,
  joint_velocity_,
  joint_effort_,
  joint_stiffness_,
  joint_damping_,
  joint_position_command_,
  joint_set_point_command_,
  joint_velocity_command_,
  joint_stiffness_command_,
  joint_damping_command_,
  joint_effort_command_,
  cart_pos_,
  cart_stiff_,
  cart_damp_,
  cart_wrench_,
  cart_pos_command_,
  cart_stiff_command_,
  cart_damp_command_,
  cart_wrench_command_;

  // NOTE:
  // joint_velocity_command is not really to command the kuka arm in velocity,
  // since it doesn't have an interface for that
  // this is used to avoid speed limit error in the kuka controller by
  // computing a fake velocity command using the received position command and
  // the current position, without smoothing.

  // Set all members to default values
  void reset();

  // KDL stuff to compute ik, gravity term, etc.
  KDL::Chain lwr_chain_;
  std::unique_ptr<KDL::ChainDynParam> f_dyn_solver_;
  KDL::JntArray joint_position_kdl_, gravity_effort_;
  KDL::Vector gravity_;

protected:

  rclcpp::Logger logger_ = rclcpp::get_logger("lwr_hw");

private:

  // Read the limits of all joints (and their stiffness/damping companions) from the URDF
  void registerJointLimits(const urdf::Model *const urdf_model);

  // Initialize all KDL members
  bool initKDLdescription(const urdf::Model *const urdf_model);

  // saturation state (previous commands), NaN means "not initialized"
  std::vector<double> prev_position_command_, prev_stiffness_command_, prev_damping_command_;

}; // class

} // namespace

#endif
