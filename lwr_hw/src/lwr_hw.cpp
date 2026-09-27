#include "lwr_hw/lwr_hw.h"

#include <algorithm>
#include <cmath>

namespace lwr_hw
{
  namespace
  {
    // split "prefix/interface" into its two parts
    void splitInterfaceName(const std::string& full_name, std::string& prefix, std::string& interface_name)
    {
      const size_t slash = full_name.rfind('/');
      prefix = full_name.substr(0, slash);
      interface_name = (slash == std::string::npos) ? std::string() : full_name.substr(slash + 1);
    }

    // equivalent of joint_limits_interface::PositionJointSaturationHandle::enforceLimits()
    double saturatePosition(double command, double position, double& prev_command,
                            double lower, double upper, double max_velocity, double period)
    {
      if (std::isnan(prev_command))
        prev_command = position;

      double min_pos = lower;
      double max_pos = upper;
      if (max_velocity < std::numeric_limits<double>::max() && period > 0.0)
      {
        const double delta_pos = max_velocity * period;
        min_pos = std::max(prev_command - delta_pos, lower);
        max_pos = std::min(prev_command + delta_pos, upper);
      }

      const double cmd = std::clamp(command, min_pos, max_pos);
      prev_command = cmd;
      return cmd;
    }
  }

  bool LWRHW::create(const hardware_interface::HardwareInfo & info)
  {
    auto param = [&info](const std::string& key, const std::string& default_value)
    {
      auto it = info.hardware_parameters.find(key);
      return (it == info.hardware_parameters.end()) ? default_value : it->second;
    };

    root_name_ = param("root_name", "");
    tip_name_ = param("tip_name", "");
    return create(param("name", "lwr"), info.original_xml);
  }

  bool LWRHW::create(std::string name, std::string urdf_string)
  {
    std::cout << "Creating a KUKA LWR 4+ called: " << name << std::endl;

    // SET NAME AND MODEL
    robot_namespace_ = name;
    cart_prefix_ = robot_namespace_ + std::string("_cart");
    urdf_string_ = urdf_string;
    logger_ = rclcpp::get_logger("lwr_hw." + robot_namespace_);

    // ALLOCATE MEMORY

    // JOINT NAMES ARE TAKEN FROM URDF NAME CONVENTION
    joint_names_.clear();
    joint_names_.push_back( robot_namespace_ + std::string("_a1_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a2_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_e1_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a3_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a4_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a5_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a6_joint") );
    // cartesian interface names (the prefix is cart_prefix_)
    cart_12_names_ = {"rot_xx", "rot_yx", "rot_zx", "pos_x",
                      "rot_xy", "rot_yy", "rot_zy", "pos_y",
                      "rot_xz", "rot_yz", "rot_zz", "pos_z"};
    cart_6_names_ = {"X", "Y", "Z", "A", "B", "C"};

    // VARIABLES
    joint_position_.resize(n_joints_);
    joint_position_prev_.resize(n_joints_);
    joint_velocity_.resize(n_joints_);
    joint_effort_.resize(n_joints_);
    joint_stiffness_.resize(n_joints_);
    joint_damping_.resize(n_joints_);
    joint_position_command_.resize(n_joints_);
    joint_set_point_command_.resize(n_joints_);
    joint_velocity_command_.resize(n_joints_);
    joint_effort_command_.resize(n_joints_);
    joint_stiffness_command_.resize(n_joints_);
    joint_damping_command_.resize(n_joints_);
    cart_pos_.resize(12);
    cart_stiff_.resize(6);
    cart_damp_.resize(6);
    cart_wrench_.resize(6);
    cart_pos_command_.resize(12);
    cart_stiff_command_.resize(6);
    cart_damp_command_.resize(6);
    cart_wrench_command_.resize(6);

    joint_lower_limits_.resize(n_joints_);
    joint_upper_limits_.resize(n_joints_);
    joint_velocity_limits_.resize(n_joints_);
    joint_lower_limits_stiffness_.resize(n_joints_);
    joint_upper_limits_stiffness_.resize(n_joints_);
    joint_velocity_limits_stiffness_.resize(n_joints_);
    joint_upper_limits_damping_.resize(n_joints_);
    joint_lower_limits_damping_.resize(n_joints_);
    joint_velocity_limits_damping_.resize(n_joints_);
    joint_effort_limits_.resize(n_joints_);

    prev_position_command_.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
    prev_stiffness_command_.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
    prev_damping_command_.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());

    // RESET VARIABLES
    reset();

    std::cout << "Parsing the URDF..." << std::endl;

    if (!urdf_model_.initString(urdf_string_))
    {
      RCLCPP_ERROR(logger_, "Error parsing URDF in lwr_hw.");
      return false;
    }

    // CHECK THAT ALL JOINTS OF THIS LWR 4+ ARM EXIST
    for (const auto& joint_name : joint_names_)
    {
      if (!urdf_model_.getJoint(joint_name))
      {
        RCLCPP_ERROR(logger_, "Joint '%s' is not in the URDF. Does the 'name' parameter (%s) match the name of the arm in the URDF?",
                     joint_name.c_str(), robot_namespace_.c_str());
        return false;
      }
    }

    std::cout << "Registering joint limits..." << std::endl;

    registerJointLimits(&urdf_model_);

    std::cout << "Initializing KDL variables..." << std::endl;

    // INIT KDL STUFF
    if (!initKDLdescription(&urdf_model_))
      return false;

    std::cout << "Succesfully created an abstract LWR 4+ ARM with interfaces to ROS control" << std::endl;
    return true;
  }

  // reset values
  void LWRHW::reset()
  {
    for (int j = 0; j < n_joints_; ++j)
    {
      joint_position_[j] = 0.0;
      joint_position_prev_[j] = 0.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 0.0;
      joint_stiffness_[j] = 0.0;
      joint_damping_[j] = 0.0;

      joint_position_command_[j] = 0.0;
      joint_set_point_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;
      joint_effort_command_[j] = 0.0;
      joint_stiffness_command_[j] = 1000.0;
      joint_damping_command_[j] = 0.7;
    }

    for(int i=0; i < 12; ++i)
    {
      cart_pos_[i] = 0.0;
      cart_pos_command_[i] = 0.0;
    }
    cart_pos_[0] = 1.0;
    cart_pos_[5] = 1.0;
    cart_pos_[10] = 1.0;
    cart_pos_command_[0] = 1.0;
    cart_pos_command_[5] = 1.0;
    cart_pos_command_[10] = 1.0;
    for(int i=0; i < 3; i++)
    {
      cart_stiff_[i] = 0.0;
      cart_stiff_[i + 3] = 0.0;
      cart_damp_[i] = 0.0;
      cart_damp_[i + 3] = 0.0;
      cart_wrench_[i] = 0.0;
      cart_wrench_[i + 3] = 0.0;
      cart_stiff_command_[i] = 800;
      cart_stiff_command_[i + 3] = 50;
      cart_damp_command_[i] = 10;
      cart_damp_command_[i + 3] = 1;
      cart_wrench_command_[i] = 0.0;
      cart_wrench_command_[i + 3] = 0.0;
    }

    current_strategy_ = JOINT_POSITION;
    commands_initialized_ = false;

    return;
  }

  std::vector<hardware_interface::StateInterface> LWRHW::exportStateInterfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for(int j=0; j < n_joints_; j++)
    {
      state_interfaces.emplace_back(joint_names_[j], hardware_interface::HW_IF_POSITION, &joint_position_[j]);
      state_interfaces.emplace_back(joint_names_[j], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[j]);
      state_interfaces.emplace_back(joint_names_[j], hardware_interface::HW_IF_EFFORT, &joint_effort_[j]);
      state_interfaces.emplace_back(joint_names_[j], HW_IF_STIFFNESS, &joint_stiffness_[j]);
      state_interfaces.emplace_back(joint_names_[j], HW_IF_DAMPING, &joint_damping_[j]);
    }

    // Now for cart variables
    for(int j=0; j < 12; ++j)
    {
      state_interfaces.emplace_back(cart_prefix_, cart_12_names_[j], &cart_pos_[j]);
    }
    for(int j=0; j < 6; ++j)
    {
      state_interfaces.emplace_back(cart_prefix_, cart_6_names_[j] + std::string("_stiffness"), &cart_stiff_[j]);
      state_interfaces.emplace_back(cart_prefix_, cart_6_names_[j] + std::string("_damping"), &cart_damp_[j]);
      state_interfaces.emplace_back(cart_prefix_, cart_6_names_[j] + std::string("_wrench"), &cart_wrench_[j]);
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> LWRHW::exportCommandInterfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(int j=0; j < n_joints_; j++)
    {
      std::cout << "\x1B[37m" << "lwr_hw: " << "Loading joint '" << joint_names_[j] << "'" << "\x1B[0m" << std::endl;

      command_interfaces.emplace_back(joint_names_[j], hardware_interface::HW_IF_POSITION, &joint_position_command_[j]);
      command_interfaces.emplace_back(joint_names_[j], hardware_interface::HW_IF_EFFORT, &joint_effort_command_[j]);
      command_interfaces.emplace_back(joint_names_[j], HW_IF_STIFFNESS, &joint_stiffness_command_[j]);
      command_interfaces.emplace_back(joint_names_[j], HW_IF_DAMPING, &joint_damping_command_[j]);
      command_interfaces.emplace_back(joint_names_[j], HW_IF_SET_POINT, &joint_set_point_command_[j]);
    }

    // Now for cart variables
    for(int j=0; j < 12; ++j)
    {
      command_interfaces.emplace_back(cart_prefix_, cart_12_names_[j], &cart_pos_command_[j]);
    }
    for(int j=0; j < 6; ++j)
    {
      command_interfaces.emplace_back(cart_prefix_, cart_6_names_[j] + std::string("_stiffness"), &cart_stiff_command_[j]);
      command_interfaces.emplace_back(cart_prefix_, cart_6_names_[j] + std::string("_damping"), &cart_damp_command_[j]);
      command_interfaces.emplace_back(cart_prefix_, cart_6_names_[j] + std::string("_wrench"), &cart_wrench_command_[j]);
    }

    return command_interfaces;
  }

  // Read the limits of the joints from the URDF model.
  // The stiffness and damping limits are read from the (dummy) joints called <joint_name>_stiffness and <joint_name>_damping.
  // TODO: register limits for cartesian variables
  void LWRHW::registerJointLimits(const urdf::Model *const urdf_model)
  {
    constexpr double inf = std::numeric_limits<double>::max();

    auto read_limits = [urdf_model](const std::string& joint_name, double& lower, double& upper, double& velocity, double* effort)
    {
      lower = -inf;
      upper = inf;
      velocity = inf;
      if (effort)
        *effort = inf;

      if (urdf_model == nullptr)
        return false;
      const auto urdf_joint = urdf_model->getJoint(joint_name);
      if (!urdf_joint || !urdf_joint->limits)
        return false;

      if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      {
        lower = urdf_joint->limits->lower;
        upper = urdf_joint->limits->upper;
      }
      if (urdf_joint->limits->velocity > 0.0)
        velocity = urdf_joint->limits->velocity;
      if (effort && urdf_joint->limits->effort > 0.0)
        *effort = urdf_joint->limits->effort;
      return true;
    };

    for (int j = 0; j < n_joints_; ++j)
    {
      read_limits(joint_names_[j], joint_lower_limits_[j], joint_upper_limits_[j], joint_velocity_limits_[j], &joint_effort_limits_[j]);
      read_limits(joint_names_[j] + std::string("_stiffness"), joint_lower_limits_stiffness_[j], joint_upper_limits_stiffness_[j], joint_velocity_limits_stiffness_[j], nullptr);
      read_limits(joint_names_[j] + std::string("_damping"), joint_lower_limits_damping_[j], joint_upper_limits_damping_[j], joint_velocity_limits_damping_[j], nullptr);
    }
  }

  void LWRHW::enforceLimits(double period)
  {
    for (int j = 0; j < n_joints_; ++j)
    {
      // position saturation (and velocity limit w.r.t. the previous command)
      joint_position_command_[j] = saturatePosition(joint_position_command_[j], joint_position_[j], prev_position_command_[j],
                                                    joint_lower_limits_[j], joint_upper_limits_[j], joint_velocity_limits_[j], period);

      // effort saturation, no effort is allowed that pushes the joint further out of its position or velocity limits
      double min_eff = -joint_effort_limits_[j];
      double max_eff = joint_effort_limits_[j];
      if (joint_position_[j] < joint_lower_limits_[j])
        min_eff = 0.0;
      else if (joint_position_[j] > joint_upper_limits_[j])
        max_eff = 0.0;
      if (joint_velocity_[j] < -joint_velocity_limits_[j])
        min_eff = 0.0;
      else if (joint_velocity_[j] > joint_velocity_limits_[j])
        max_eff = 0.0;
      joint_effort_command_[j] = std::clamp(joint_effort_command_[j], min_eff, max_eff);

      // stiffness and damping saturation
      joint_stiffness_command_[j] = saturatePosition(joint_stiffness_command_[j], joint_stiffness_[j], prev_stiffness_command_[j],
                                                     joint_lower_limits_stiffness_[j], joint_upper_limits_stiffness_[j], joint_velocity_limits_stiffness_[j], period);
      joint_damping_command_[j] = saturatePosition(joint_damping_command_[j], joint_damping_[j], prev_damping_command_[j],
                                                   joint_lower_limits_damping_[j], joint_upper_limits_damping_[j], joint_velocity_limits_damping_[j], period);
    }
  }

  void LWRHW::resetCommandsOnSwitch()
  {
    for (int j = 0; j < n_joints_; ++j)
    {
      ///semantic Zero
      joint_position_command_[j] = joint_position_[j];
      joint_effort_command_[j] = 0.0;

      ///reset joint limits saturation
      prev_position_command_[j] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  void LWRHW::initCommandsFromState()
  {
    for (int j = 0; j < n_joints_; ++j)
    {
      joint_position_command_[j] = joint_position_[j];
      joint_set_point_command_[j] = joint_position_[j];
    }
    commands_initialized_ = true;
  }

  // Init KDL stuff
  bool LWRHW::initKDLdescription(const urdf::Model *const urdf_model)
  {
    // KDL code to compute f_dyn(q)
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
    {
        RCLCPP_ERROR(logger_, "Failed to construct kdl tree");
        return false;
    }

    std::cout << "LWR kinematic successfully parsed with "
              << kdl_tree.getNrOfJoints()
              << " joints, and "
              << kdl_tree.getNrOfSegments()
              << " segments." << std::endl;

    // Get the info from the hardware parameters
    std::string root_name = root_name_;
    if( root_name.empty() )
      root_name = kdl_tree.getRootSegment()->first; // default

    std::string tip_name = tip_name_;
    if( tip_name.empty() )
      tip_name = robot_namespace_ + std::string("_7_link"); ; // default

    std::cout << "Using root: " << root_name << " and tip: " << tip_name << std::endl;

    // this depends on how the world frame is set, in all our setups, world has always positive z pointing up.
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // Extract the chain from the tree
    if(!kdl_tree.getChain(root_name, tip_name, lwr_chain_))
    {
        RCLCPP_ERROR(logger_, "Failed to get KDL chain from tree: %s --> %s", root_name.c_str(), tip_name.c_str());
        return false;
    }

    RCLCPP_INFO(logger_, "Number of segments: %d", lwr_chain_.getNrOfSegments());
    RCLCPP_INFO(logger_, "Number of joints in chain: %d", lwr_chain_.getNrOfJoints());

    f_dyn_solver_.reset(new KDL::ChainDynParam(lwr_chain_,gravity_));

    joint_position_kdl_ = KDL::JntArray(lwr_chain_.getNrOfJoints());
    gravity_effort_ = KDL::JntArray(lwr_chain_.getNrOfJoints());

    return true;
  }

  bool LWRHW::prepareSwitch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) const
  {
    (void)stop_interfaces;
    int counter_position = 0, counter_effort = 0, counter_cartesian = 0;

    for (const auto& full_name : start_interfaces)
    {
      std::string prefix, interface_name;
      splitInterfaceName(full_name, prefix, interface_name);

      if (prefix == cart_prefix_)
      {
        counter_cartesian = 1;
        continue;
      }
      if (std::find(joint_names_.begin(), joint_names_.end(), prefix) == joint_names_.end())
        continue; // not ours

      // If any of the controllers to start works on a velocity interface, the switch can't be done.
      if (interface_name == hardware_interface::HW_IF_VELOCITY)
      {
        std::cout << "The given controllers to start work on a velocity joint interface, and this robot does not have such an interface. "
                  << "The switch can't be done" << std::endl;
        return false;
      }
      if (interface_name == hardware_interface::HW_IF_POSITION)
        counter_position = 1;
      else
        counter_effort = 1; // effort, stiffness, damping and set_point
    }

    if( (counter_position+counter_effort+counter_cartesian)>1)
    {
      std::cout << "OOPS! The control mode of the LWR is selected from the command interfaces claimed by the controllers. "
                << "Controllers claiming joint position, joint impedance (effort/stiffness/damping/set_point) and cartesian "
                << "command interfaces can't be started together, so we can't switch"
                << std::endl;
      return false;
    }

    return true;
  }

  void LWRHW::doSwitch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
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
      setControlStrategy(desired_strategy);
      std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    }
  }

  LWRHW::ControlStrategy LWRHW::getNewControlStrategy(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces, ControlStrategy default_control_strategy) const
  {
    (void)stop_interfaces;
    ControlStrategy desired_strategy = default_control_strategy;

    // NOTE that this allows to switch only based on the first command interface of the list
    for (const auto& full_name : start_interfaces)
    {
      std::string prefix, interface_name;
      splitInterfaceName(full_name, prefix, interface_name);

      if (prefix == cart_prefix_)
      {
        std::cout << "Request to switch to a cartesian command interface (CARTESIAN_IMPEDANCE)" << std::endl;
        desired_strategy = CARTESIAN_IMPEDANCE;
        break;
      }
      if (std::find(joint_names_.begin(), joint_names_.end(), prefix) == joint_names_.end())
        continue;

      if (interface_name == hardware_interface::HW_IF_POSITION)
      {
        std::cout << "Request to switch to a joint position command interface (JOINT_POSITION)" << std::endl;
        desired_strategy = JOINT_POSITION;
        break;
      }
      else if (interface_name == hardware_interface::HW_IF_EFFORT || interface_name == HW_IF_STIFFNESS ||
               interface_name == HW_IF_DAMPING || interface_name == HW_IF_SET_POINT)
      {
        std::cout << "Request to switch to a joint effort/stiffness/damping/set_point command interface (JOINT_IMPEDANCE)" << std::endl;
        desired_strategy = JOINT_IMPEDANCE;
        break;
      }
    }

    return desired_strategy;
  }

}
