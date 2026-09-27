// LWR sim class
#include "lwr_hw/lwr_hw_gazebo.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace lwr_hw
{

hardware_interface::CallbackReturn LWRHWGazebo::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  return CallbackReturn::SUCCESS;
}

bool LWRHWGazebo::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr sdf)
{
  (void)sdf;
  std::cout << "lwr_hw" << "Loading lwr_hw plugin" << std::endl;

  nh_ = model_nh;
  parent_model_ = parent_model;
  parent_set_ = true;

  // Error message if the model couldn't be found
  if (!parent_model_)
  {
    RCLCPP_ERROR(logger_, "parent model is NULL");
    return false;
  }

  // Load the LWRHW abstraction to interface the controllers with the gazebo model
  if (!create(hardware_info))
  {
    RCLCPP_FATAL(logger_, "Could not create the LWR 4+ robot simulation interface");
    return false;
  }

  gazebo::physics::JointPtr joint;
  for(int j=0; j < n_joints_; j++)
  {
    joint = parent_model_->GetJoint(joint_names_[j]);
    if (!joint)
    {
      std::cout << "This robot has a joint named \"" << joint_names_[j]
        << "\" which is not in the gazebo model." << std::endl;
      return false;
    }
    sim_joints_.push_back(joint);
  }

  RCLCPP_INFO(logger_, "Loaded lwr_hw for %s.", robot_namespace_.c_str());
  return true;
}

std::vector<hardware_interface::StateInterface> LWRHWGazebo::export_state_interfaces()
{
  return exportStateInterfaces();
}

std::vector<hardware_interface::CommandInterface> LWRHWGazebo::export_command_interfaces()
{
  return exportCommandInterfaces();
}

hardware_interface::return_type LWRHWGazebo::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return prepareSwitch(start_interfaces, stop_interfaces) ? hardware_interface::return_type::OK
                                                          : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type LWRHWGazebo::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  doSwitch(start_interfaces, stop_interfaces);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LWRHWGazebo::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  for(int j=0; j < n_joints_; ++j)
  {
    joint_position_prev_[j] = joint_position_[j];
    joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                            sim_joints_[j]->Position(0));
    joint_position_kdl_(j) = joint_position_[j];
    // derivate velocity as in the real hardware instead of reading it from simulation
    if (period.seconds() > 0.0)
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j])/period.seconds(), joint_velocity_[j], 0.2);
    joint_effort_[j] = sim_joints_[j]->GetForce((int)(0));
    joint_stiffness_[j] = joint_stiffness_command_[j];
  }

  if (!commands_initialized_)
    initCommandsFromState();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LWRHWGazebo::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  enforceLimits(period.seconds());

  switch (getControlStrategy())
  {

    case JOINT_POSITION:
      for(int j=0; j < n_joints_; j++)
      {
        // according to the gazebo_ros_control plugin, this must *not* be called if SetForce is going to be called
        // but should be called when SetPostion is going to be called
        // so enable this when I find the SetMaxForce reset.
        // sim_joints_[j]->SetMaxForce(0, joint_effort_limits_[j]);
        sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
      }
      break;

    case CARTESIAN_IMPEDANCE:
      if(time.get_clock_type() == lastT_.get_clock_type() && time-lastT_ < ts_)
          break;
      lastT_ = time;
      RCLCPP_WARN(logger_, "CARTESIAN IMPEDANCE NOT AVAILABLE IN GAZEBO, PRINTING THE COMMANDED VALUES:");
      std::cout << "Notice that this printing is done only every " << ts_.seconds() << " seconds: to change this, change ts_ in lwr_hw_gazebo.hpp..." << std::endl;
      std::cout << "cart_pos_command_ = | ";
      for(int i=0; i < 12; ++i)
          std::cout << cart_pos_command_[i] << " | ";
      std::cout << std::endl << "cart_stiff_command_ = | ";
      for(int i=0; i < 6; i++)
          std::cout << cart_stiff_command_[i] << " | ";
      std::cout << std::endl << "cart_damp_command_ = | ";
      for(int i=0; i < 6; i++)
          std::cout << cart_damp_command_[i] << " | ";
      std::cout << std::endl << "cart_wrench_command_ = | ";
      for(int i=0; i < 6; i++)
          std::cout << cart_wrench_command_[i] << " | ";
      std::cout << std::endl << "Here, the call to doCartesianImpedanceControl() is done" << std::endl;
      break;

    case JOINT_IMPEDANCE:
      // compute the gracity term
      f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

      for(int j=0; j < n_joints_; j++)
      {
        // replicate the joint impedance control strategy
        // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
        const double stiffness_effort = 0.0;//10.0*( joint_position_command_[j] - joint_position_[j] ); // joint_stiffness_command_[j]*( joint_position_command_[j] - joint_position_[j] );
        //double damping_effort = joint_damping_command_[j]*( joint_velocity_[j] );
        const double effort = stiffness_effort + joint_effort_command_[j] + gravity_effort_(j);
        sim_joints_[j]->SetForce(0, effort);
      }
      break;

    case GRAVITY_COMPENSATION:
      RCLCPP_WARN(logger_, "CARTESIAN IMPEDANCE NOT IMPLEMENTED");
      break;

    default:
      break;
  }

  return hardware_interface::return_type::OK;
}

} // namespace

// Register this plugin with gazebo_ros2_control
PLUGINLIB_EXPORT_CLASS(lwr_hw::LWRHWGazebo, gazebo_ros2_control::GazeboSystemInterface)
