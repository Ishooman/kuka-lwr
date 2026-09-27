// the lwr hw fril interface
#include "lwr_hw/lwr_hw_fril.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace lwr_hw
{

hardware_interface::CallbackReturn LWRHWFRIL::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // the general robot description, the lwr class will take care of parsing what's useful to itself
  if (!create(info))
  {
    RCLCPP_FATAL(logger_, "Could not create the LWR 4+ real interface");
    return CallbackReturn::ERROR;
  }

  auto it = info.hardware_parameters.find("file");
  setInitFile(it == info.hardware_parameters.end() ? std::string("") : it->second);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LWRHWFRIL::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  // start the real lwr
  if(!init())
  {
    RCLCPP_FATAL(logger_, "Could not initialize robot real interface");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LWRHWFRIL::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  std::cerr<<"Stopping LWR..."<<std::endl;
  stop();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LWRHWFRIL::export_state_interfaces()
{
  return exportStateInterfaces();
}

std::vector<hardware_interface::CommandInterface> LWRHWFRIL::export_command_interfaces()
{
  return exportCommandInterfaces();
}

hardware_interface::return_type LWRHWFRIL::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return prepareSwitch(start_interfaces, stop_interfaces) ? hardware_interface::return_type::OK
                                                          : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type LWRHWFRIL::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  doSwitch(start_interfaces, stop_interfaces);
  return hardware_interface::return_type::OK;
}

void LWRHWFRIL::stop()
{
  if (device_)
    device_->StopRobot();
  return;
}

bool LWRHWFRIL::init()
{
  if( !(file_set_) || init_file_.empty() )
  {
    std::cout << "Did you forget to set the init file?" << std::endl
              << "You must do that before init()" << std::endl
              << "Exiting..." << std::endl;
    return false;
  }

  // construct a low-level lwr
  device_.reset( new FastResearchInterface( init_file_.c_str() ) );

  ResultValue	=	device_->StartRobot( FRI_CONTROL_POSITION );
  if (ResultValue != EOK)
  {
    std::cout << "An error occurred during starting up the robot...\n" << std::endl;
    return false;
  }

  return true;
}

hardware_interface::return_type LWRHWFRIL::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  float msrJntPos[n_joints_];
  float msrJntTrq[n_joints_];

  device_->GetMeasuredJointPositions( msrJntPos );
  device_->GetMeasuredJointTorques( msrJntTrq );

  for (int j = 0; j < n_joints_; j++)
  {
    joint_position_prev_[j] = joint_position_[j];
    joint_position_[j] = (double)msrJntPos[j];
    joint_position_kdl_(j) = joint_position_[j];
    joint_effort_[j] = (double)msrJntTrq[j];
    if (period.seconds() > 0.0)
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.seconds(), joint_velocity_[j], 0.2);
    joint_stiffness_[j] = joint_stiffness_command_[j];
    joint_damping_[j] = joint_damping_command_[j];
  }

  if (!commands_initialized_)
    initCommandsFromState();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LWRHWFRIL::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  enforceLimits(period.seconds());

  // ensure the robot is powered and it is in control mode, almost like the isMachineOk() of Standford
  if ( device_->IsMachineOK() )
  {
    device_->WaitForKRCTick();

    switch (getControlStrategy())
    {

      case JOINT_POSITION:

        // Ensure the robot is in this mode
        if( (device_->GetCurrentControlScheme() == FRI_CONTROL_POSITION) )
        {
           float newJntPosition[n_joints_];
           for (int j = 0; j < n_joints_; j++)
           {
             newJntPosition[j] = (float)joint_position_command_[j];
           }
           device_->SetCommandedJointPositions(newJntPosition);
        }
        break;

      case CARTESIAN_IMPEDANCE:
        break;

       case JOINT_IMPEDANCE:

        // Ensure the robot is in this mode
        if( (device_->GetCurrentControlScheme() == FRI_CONTROL_JNT_IMP) )
        {
         float newJntPosition[n_joints_];
         float newJntStiff[n_joints_];
         float newJntDamp[n_joints_];
         float newJntAddTorque[n_joints_];

         // WHEN THE URDF MODEL IS PRECISE
         // 1. compute the gracity term
         // f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

         // 2. read gravity term from FRI and add it with opposite sign and add the URDF gravity term
         // newJntAddTorque = gravity_effort_  - device_->getF_DYN??

          for(int j=0; j < n_joints_; j++)
          {
            newJntPosition[j] = (float)joint_set_point_command_[j];
            newJntAddTorque[j] = (float)joint_effort_command_[j];
            newJntStiff[j] = (float)joint_stiffness_command_[j];
            newJntDamp[j] = (float)joint_damping_command_[j];
          }
          device_->SetCommandedJointStiffness(newJntStiff);
          device_->SetCommandedJointPositions(newJntPosition);
          device_->SetCommandedJointDamping(newJntDamp);
          device_->SetCommandedJointTorques(newJntAddTorque);
        }
        break;

       case GRAVITY_COMPENSATION:
         break;

       default:
         break;
     }
  }
  return hardware_interface::return_type::OK;
}

void LWRHWFRIL::doSwitch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{

  ResultValue	=	device_->StopRobot();
  if (ResultValue != EOK)
  {
      std::cout << "An error occurred during stopping the robot, couldn't switch mode...\n" << std::endl;
      return;
  }

  // at this point, we now that there is only one control mode requested
  ControlStrategy desired_strategy = JOINT_POSITION; // default

  desired_strategy = getNewControlStrategy(start_interfaces,stop_interfaces,desired_strategy);

  // only allow joint position and joint impedance control strategies, otherwise set the default (JOINT_POSITION) strategy
  if(desired_strategy != JOINT_POSITION && desired_strategy != JOINT_IMPEDANCE)
      desired_strategy = JOINT_POSITION;

  resetCommandsOnSwitch();

  if(desired_strategy == getControlStrategy())
  {
    std::cout << "The ControlStrategy didn't changed, it is already: " << getControlStrategy() << std::endl;
  }
  else
  {
    switch( desired_strategy )
    {
      case JOINT_POSITION:
        ResultValue = device_->StartRobot( FRI_CONTROL_POSITION );
        if (ResultValue != EOK)
        {
          std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_POSITION...\n" << std::endl;
          return;
        }
        break;
       case JOINT_IMPEDANCE:
        ResultValue = device_->StartRobot( FRI_CONTROL_JNT_IMP );
        if (ResultValue != EOK)
        {
          std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_IMPEDANCE...\n" << std::endl;
          return;
        }
        break;
       default:
        break;
    }

    // if sucess during the switch in FRI, set the ROS strategy
    setControlStrategy(desired_strategy);

    std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_hw::LWRHWFRIL, hardware_interface::SystemInterface)
