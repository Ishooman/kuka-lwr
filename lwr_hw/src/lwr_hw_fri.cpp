// the lwr hw fri interface
#include "lwr_hw/lwr_hw_fri.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace lwr_hw
{

LWRHWFRI::~LWRHWFRI()
{
  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn LWRHWFRI::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // get params or give default values
  auto param = [&info](const std::string& key, const std::string& default_value)
  {
    auto it = info.hardware_parameters.find(key);
    return (it == info.hardware_parameters.end() || it->second.empty()) ? default_value : it->second;
  };

  // the general robot description, the lwr class will take care of parsing what's useful to itself
  if (!create(info))
  {
    RCLCPP_FATAL(logger_, "Could not create the LWR 4+ real interface");
    return CallbackReturn::ERROR;
  }

  setPort(std::stoi(param("port", "49939")));
  setIP(param("ip", "192.168.0.10"));
  estop_topic_ = param("estop_topic", std::string("/") + robot_namespace_ + std::string("/emergency_stop"));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LWRHWFRI::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  // advertise the e-stop topic
  estop_node_ = std::make_shared<rclcpp::Node>(robot_namespace_ + "_hw_fri",
                                               rclcpp::NodeOptions().start_parameter_services(false).use_global_arguments(false));
  estop_sub_ = estop_node_->create_subscription<std_msgs::msg::Bool>(
    estop_topic_, 1, [this](const std_msgs::msg::Bool::SharedPtr e_stop_msg) { isStopPressed_ = e_stop_msg->data; });
  estop_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  estop_executor_->add_node(estop_node_);
  estop_thread_.reset(new std::thread([this]() { estop_executor_->spin(); }));

  // construct and start the real lwr
  if(!init())
  {
    RCLCPP_FATAL(logger_, "Could not initialize robot real interface");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Sampling time on robot: %f. Set the controller manager update_rate accordingly.", sampling_rate_);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LWRHWFRI::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  stopKRCComm_ = true;
  if (KRCCommThread_ && KRCCommThread_->joinable())
    KRCCommThread_->join();
  KRCCommThread_.reset();

  if (estop_executor_)
    estop_executor_->cancel();
  if (estop_thread_ && estop_thread_->joinable())
    estop_thread_->join();
  estop_thread_.reset();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LWRHWFRI::export_state_interfaces()
{
  return exportStateInterfaces();
}

std::vector<hardware_interface::CommandInterface> LWRHWFRI::export_command_interfaces()
{
  return exportCommandInterfaces();
}

hardware_interface::return_type LWRHWFRI::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return prepareSwitch(start_interfaces, stop_interfaces) ? hardware_interface::return_type::OK
                                                          : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type LWRHWFRI::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  doSwitch(start_interfaces, stop_interfaces);
  return hardware_interface::return_type::OK;
}

bool LWRHWFRI::init()
{
  if( !(port_set_) || !(ip_set_) )
  {
    std::cout << "Did you forget to set the port/ip?" << std::endl << "You must do that before init()" << std::endl << "Exiting..." << std::endl;
    return false;
  }

  // construct a low-level lwr
  device_.reset( new friRemote( port_, const_cast<char*>(hintToRemoteHost_.c_str()) ) );

  // initialize FRI values
  lastQuality_ = FRI_QUALITY_BAD;
  lastCtrlScheme_ = FRI_CTRL_OTHER;

  std::cout << "Opening FRI Version "
    << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR
    << " Interface for LWR ROS server" << std::endl;

  std::cout << "Checking if the robot is Stopped..." << std::endl;
  if( device_->getState() == FRI_STATE_OFF )
  {
    std::cout << "Please, start the KRL script now." << std::endl;
  }
  stopKRCComm_ = false;
  KRCCommThread_.reset( new std::thread( &LWRHWFRI::KRCCommThreadCallback,this ) );

  startFRI();

  std::cout << "Ready, FRI has been started!" << std::endl;
  std::cout << "FRI Status:\n" << device_->getMsrBuf().intf << std::endl;
  sampling_rate_ = device_->getSampleTime();
  std::cout << "Sampling Rate: " << sampling_rate_ << std::endl;

  return true;
}

hardware_interface::return_type LWRHWFRI::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  for (int j = 0; j < n_joints_; j++)
  {
    joint_position_prev_[j] = joint_position_[j];
    joint_position_[j] = device_->getMsrMsrJntPosition()[j];
    joint_position_kdl_(j) = joint_position_[j];
    joint_effort_[j] = device_->getMsrJntTrq()[j];
    if (period.seconds() > 0.0)
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.seconds(), joint_velocity_[j], 0.2);
    joint_stiffness_[j] = joint_stiffness_command_[j];
    joint_damping_[j] = joint_damping_command_[j];
  }
  for(int j = 0; j < 12; j++)
  {
      cart_pos_[j] = device_->getMsrCartPosition()[j];
  }
  for(int j = 0; j < 6; j++)
  {
      cart_stiff_[j] = cart_stiff_command_[j];
      cart_damp_[j] = cart_damp_command_[j];
      cart_wrench_[j] = cart_wrench_command_[j];
  }

  if (!commands_initialized_)
    initCommandsFromState();

  // Handle the emergency stop
  if( isStopPressed_ )
  {
    if( wasStopHandled_ )
    {
      RCLCPP_WARN(logger_, "E-STOP HAS BEEN PRESSED: The controller commands are ignored and the measured position is held until you release the E-Stop");
      RCLCPP_WARN(logger_, "HOW TO RELEASE E-STOP: ros2 topic pub -r 10 %s std_msgs/msg/Bool 'data: false'", estop_topic_.c_str());
      RCLCPP_WARN(logger_, "NOTE: the controllers are NOT restarted, restart them (ros2 control switch_controllers) before releasing the E-Stop");
    }
    wasStopHandled_ = false;
  }
  else
  {
    wasStopHandled_ = true;
  }

  return hardware_interface::return_type::OK;
}

void LWRHWFRI::holdMeasuredPosition()
{
  for (int j = 0; j < n_joints_; ++j)
  {
    joint_position_command_[j] = joint_position_[j];
    joint_set_point_command_[j] = joint_position_[j];
    joint_effort_command_[j] = 0.0;
  }
  for (int i = 0; i < 12; ++i)
    cart_pos_command_[i] = cart_pos_[i];
  for (int i = 0; i < 6; ++i)
    cart_wrench_command_[i] = 0.0;
}

hardware_interface::return_type LWRHWFRI::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;

  if( isStopPressed_ )
    holdMeasuredPosition();

  enforceLimits(period.seconds());

  float newJntPosition[n_joints_];
  float newJntStiff[n_joints_];
  float newJntDamp[n_joints_];
  float newJntAddTorque[n_joints_];
  float newCartPos[12];
  float newCartStiff[6];
  float newCartDamp[6];
  float newAddFT[6];

  switch (getControlStrategy())
  {
    case JOINT_POSITION:
      for (int j = 0; j < n_joints_; j++)
      {
        newJntPosition[j] = joint_position_command_[j];
      }
      device_->doPositionControl(newJntPosition, false);
      break;

    case CARTESIAN_IMPEDANCE:
      for(int i=0; i < 12; ++i)
      {
        newCartPos[i] = cart_pos_command_[i];
      }
      for(int i=0; i < 6; i++)
      {
        newCartStiff[i] = cart_stiff_command_[i];
        newCartDamp[i] = cart_damp_command_[i];
        newAddFT[i] = cart_wrench_command_[i];
      }
      device_->doCartesianImpedanceControl(newCartPos, newCartStiff, newCartDamp, newAddFT, NULL, false);
      break;

    case JOINT_IMPEDANCE:
      for(int j=0; j < n_joints_; j++)
      {
        newJntPosition[j] = joint_set_point_command_[j];
        newJntAddTorque[j] = joint_effort_command_[j];
        newJntStiff[j] = joint_stiffness_command_[j];
        newJntDamp[j] = joint_damping_command_[j];
      }
      device_->doJntImpedanceControl(newJntPosition, newJntStiff, newJntDamp, newJntAddTorque, false);
      break;

   case JOINT_EFFORT:
      for(int j=0; j < n_joints_; j++)
      {
          newJntAddTorque[j] = joint_effort_command_[j];
          newJntStiff[j] = 0.0;
      }
      // mirror the position
      device_->doJntImpedanceControl(device_->getMsrMsrJntPosition(), newJntStiff, NULL, newJntAddTorque, false);
      break;

    case JOINT_STIFFNESS:
      for(int j=0; j < n_joints_; j++)
      {
        newJntPosition[j] = joint_set_point_command_[j];
        newJntStiff[j] = joint_stiffness_command_[j];
      }
      device_->doJntImpedanceControl(newJntPosition, newJntStiff, NULL, NULL, false);
      break;

    case GRAVITY_COMPENSATION:
      device_->doJntImpedanceControl(device_->getMsrMsrJntPosition(), NULL, NULL, NULL, false);
      break;
  }
  return hardware_interface::return_type::OK;
}

void LWRHWFRI::doSwitch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
  // at this point, we now that there is only one control mode requested
  ControlStrategy desired_strategy = JOINT_POSITION; // default

  desired_strategy = getNewControlStrategy(start_interfaces,stop_interfaces,desired_strategy);

  resetCommandsOnSwitch();

  if(desired_strategy == getControlStrategy())
  {
    std::cout << "The ControlStrategy didn't change, it is already: " << getControlStrategy() << std::endl;
  }
  else
  {
    stopFRI();

    // send to KRL the new strategy
    if( desired_strategy == JOINT_POSITION )
      device_->setToKRLInt(0, JOINT_POSITION);
    else if( desired_strategy == JOINT_IMPEDANCE)
      device_->setToKRLInt(0, JOINT_IMPEDANCE);
    else if( desired_strategy == CARTESIAN_IMPEDANCE)
      device_->setToKRLInt(0, CARTESIAN_IMPEDANCE);


    startFRI();

    setControlStrategy(desired_strategy);
    std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
  }
}

void LWRHWFRI::KRCCommThreadCallback()
{
  while(!stopKRCComm_)
  {
    device_->doDataExchange();
  }
  return;
}

void LWRHWFRI::startFRI()
{
  // wait until FRI enters in command mode
  // std::cout << "Waiting for good communication quality..." << std::endl;
  // while( device_->getQuality() != FRI_QUALITY_OK ){};
  device_->setToKRLInt(1, 1);
  device_->doDataExchange();

  // std::cout << "Waiting for command mode..." << std::endl;
  // while ( device_->getFrmKRLInt(1) != 1 )
  // {
    // std::cout << "device_->getState(): " << device_->getState() << std::endl;
    // device_->setToKRLInt(1, 1);
    // usleep(1000000);
  // }
  return;
}

void LWRHWFRI::stopFRI()
{
  // wait until FRI enters in command mode
  device_->setToKRLInt(1, 0);
  std::cout << "Waiting for monitor mode..." << std::endl;
  while ( device_->getFrmKRLInt(1) != 0 ){}
  // {
    // std::cout << "device_->getState(): " << device_->getState() << std::endl;
    // std::cout << "Waiting for monitor mode..." << std::endl;
    // device_->setToKRLInt(1, 0);
    // usleep(1000000);
  // }
  return;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_hw::LWRHWFRI, hardware_interface::SystemInterface)
