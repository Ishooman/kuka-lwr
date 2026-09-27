#include <lwr_controllers/KinematicChainControllerBase.h>

namespace controller_interface
{
namespace
{
    template<class T>
    T * findInterface(std::vector<T> & interfaces, const std::string & prefix, const std::string & interface_name)
    {
        for (auto & interface : interfaces)
        {
            if (interface.get_prefix_name() == prefix && interface.get_interface_name() == interface_name)
                return &interface;
        }
        return nullptr;
    }
}

CallbackReturn KinematicChainControllerBase::on_init()
{
    try
    {
        auto_declare<std::string>("root_name", "");
        auto_declare<std::string>("tip_name", "");
        auto_declare<std::string>("robot_description", "");
        auto_declare<std::string>("robot_description_topic", "/robot_description");
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(logger(), "Exception thrown during init stage with message: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

InterfaceConfiguration KinematicChainControllerBase::command_interface_configuration() const
{
    InterfaceConfiguration config;
    config.type = interface_configuration_type::INDIVIDUAL;

    if (command_type_ == CommandType::NONE)
    {
        config.type = interface_configuration_type::NONE;
        return config;
    }

    for (const auto & joint : chain_joint_names_)
    {
        if (command_type_ == CommandType::POSITION)
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        else
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);

        if (claim_impedance_interfaces_)
        {
            config.names.push_back(joint + "/" + lwr_hw::HW_IF_STIFFNESS);
            config.names.push_back(joint + "/" + lwr_hw::HW_IF_DAMPING);
            config.names.push_back(joint + "/" + lwr_hw::HW_IF_SET_POINT);
        }
    }
    return config;
}

InterfaceConfiguration KinematicChainControllerBase::state_interface_configuration() const
{
    InterfaceConfiguration config;
    config.type = interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : chain_joint_names_)
    {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
        if (claim_impedance_interfaces_)
        {
            config.names.push_back(joint + "/" + lwr_hw::HW_IF_STIFFNESS);
            config.names.push_back(joint + "/" + lwr_hw::HW_IF_DAMPING);
        }
    }
    return config;
}

double KinematicChainControllerBase::getParamDouble(const std::string & name, double default_value)
{
    if (!get_node()->has_parameter(name))
        get_node()->declare_parameter(name, default_value);

    const rclcpp::Parameter param = get_node()->get_parameter(name);
    switch (param.get_type())
    {
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return param.as_double();
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return static_cast<double>(param.as_int());
        default:
            return default_value;
    }
}

std::string KinematicChainControllerBase::getParamString(const std::string & name, const std::string & default_value)
{
    if (!get_node()->has_parameter(name))
        get_node()->declare_parameter(name, default_value);

    const rclcpp::Parameter param = get_node()->get_parameter(name);
    return (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) ? param.as_string() : default_value;
}

std::string KinematicChainControllerBase::getRobotDescription()
{
    std::string xml_string = getParamString("robot_description", "");
    if (!xml_string.empty())
        return xml_string;

    // wait for the latched robot description topic, using a private node so it doesn't depend on the controller manager executor
    const std::string topic = getParamString("robot_description_topic", "/robot_description");
    auto node = std::make_shared<rclcpp::Node>(
        std::string(get_node()->get_name()) + "_robot_description_listener", get_node()->get_namespace(),
        rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false).use_global_arguments(false));
    auto sub = node->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).transient_local().reliable(),
        [&xml_string](const std_msgs::msg::String::SharedPtr msg) { xml_string = msg->data; });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    RCLCPP_INFO(logger(), "Waiting for the robot description on topic %s", topic.c_str());
    const auto start = std::chrono::steady_clock::now();
    while (xml_string.empty() && rclcpp::ok() && std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        executor.spin_some(std::chrono::milliseconds(100));
        if (xml_string.empty())
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return xml_string;
}

CallbackReturn KinematicChainControllerBase::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    // get URDF and name of root and tip from the parameter server
    std::string root_name = getParamString("root_name", "");
    std::string tip_name = getParamString("tip_name", "");

    if (root_name.empty())
    {
        RCLCPP_ERROR_STREAM(logger(), "KinematicChainControllerBase: No root name found on parameter server ("<<get_node()->get_namespace() << "/" << get_node()->get_name()<<"/root_name)");
        return CallbackReturn::ERROR;
    }

    if (tip_name.empty())
    {
        RCLCPP_ERROR_STREAM(logger(), "KinematicChainControllerBase: No tip name found on parameter server ("<<get_node()->get_namespace() << "/" << get_node()->get_name()<<"/tip_name)");
        return CallbackReturn::ERROR;
    }

    // Get the gravity vector (direction and magnitude)
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // Construct an URDF model from the xml string
    std::string xml_string = getRobotDescription();

    if (xml_string.size() == 0)
    {
        RCLCPP_ERROR(logger(), "Unable to load robot model from the robot_description parameter or topic");
        return CallbackReturn::ERROR;
    }

    RCLCPP_DEBUG(logger(), "robot_description content\n%s", xml_string.c_str());

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        RCLCPP_ERROR(logger(), "Failed to parse urdf file");
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger(), "Successfully parsed urdf file");

    KDL::Tree kdl_tree_;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
        RCLCPP_ERROR(logger(), "Failed to construct kdl tree");
        return CallbackReturn::ERROR;
    }

    // Populate the KDL chain
    kdl_chain_ = KDL::Chain();
    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        RCLCPP_ERROR_STREAM(logger(), "Failed to get KDL chain from tree: ");
        RCLCPP_ERROR_STREAM(logger(), "  "<<root_name<<" --> "<<tip_name);
        RCLCPP_ERROR_STREAM(logger(), "  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        RCLCPP_ERROR_STREAM(logger(), "  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        RCLCPP_ERROR_STREAM(logger(), "  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
          RCLCPP_ERROR_STREAM(logger(), "    "<<(*it).first);

        return CallbackReturn::ERROR;
    }

    RCLCPP_DEBUG(logger(), "Number of segments: %d", kdl_chain_.getNrOfSegments());
    RCLCPP_DEBUG(logger(), "Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

    // Parsing joint limits from urdf model along kdl chain
    urdf::LinkConstSharedPtr link_ = model.getLink(tip_name);
    urdf::JointConstSharedPtr joint_;
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
    int index;

    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
    {
        joint_ = model.getJoint(link_->parent_joint->name);
        index = kdl_chain_.getNrOfJoints() - i - 1;

        if (joint_->type == urdf::Joint::FIXED)
        {
            // fixed joints are not in the KDL joint list
            i--;
        }
        else
        {
            RCLCPP_INFO(logger(), "Getting limits for joint: %s", joint_->name.c_str());
            joint_limits_.min(index) = joint_->limits->lower;
            joint_limits_.max(index) = joint_->limits->upper;
            joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;
        }

        link_ = model.getLink(link_->getParent()->name);
    }

    // Get the names of all of the joints in the chain
    chain_joint_names_.clear();
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    {
        if ( it->getJoint().getType() != KDL::Joint::None )
        {
            chain_joint_names_.push_back(it->getJoint().getName());
            RCLCPP_DEBUG(logger(), "%s", it->getJoint().getName().c_str() );
        }
    }

    RCLCPP_DEBUG(logger(), "Number of joints in handle = %lu", chain_joint_names_.size() );

    joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
    joint_des_states_.resize(kdl_chain_.getNrOfJoints());

    return CallbackReturn::SUCCESS;
}

CallbackReturn KinematicChainControllerBase::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    // Get joint handles for all of the joints in the chain
    if (!getHandles())
        return CallbackReturn::ERROR;

    return CallbackReturn::SUCCESS;
}

CallbackReturn KinematicChainControllerBase::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    joint_handles_.clear();
    joint_stiffness_handles_.clear();
    joint_damping_handles_.clear();
    joint_set_point_handles_.clear();
    return CallbackReturn::SUCCESS;
}

bool KinematicChainControllerBase::getHandles()
{
    joint_handles_.clear();
    joint_stiffness_handles_.clear();
    joint_damping_handles_.clear();
    joint_set_point_handles_.clear();

    for (const auto & joint : chain_joint_names_)
    {
        JointHandle handle;
        handle.name = joint;
        handle.position = findInterface(state_interfaces_, joint, hardware_interface::HW_IF_POSITION);
        handle.velocity = findInterface(state_interfaces_, joint, hardware_interface::HW_IF_VELOCITY);
        handle.effort = findInterface(state_interfaces_, joint, hardware_interface::HW_IF_EFFORT);
        if (!handle.position || !handle.velocity || !handle.effort)
        {
            RCLCPP_ERROR(logger(), "Could not get the state interfaces of joint %s", joint.c_str());
            return false;
        }

        if (command_type_ != CommandType::NONE)
        {
            const std::string command_interface = (command_type_ == CommandType::POSITION) ? hardware_interface::HW_IF_POSITION
                                                                                           : hardware_interface::HW_IF_EFFORT;
            handle.command = findInterface(command_interfaces_, joint, command_interface);
            if (!handle.command)
            {
                RCLCPP_ERROR(logger(), "Could not get the %s command interface of joint %s", command_interface.c_str(), joint.c_str());
                return false;
            }
        }
        joint_handles_.push_back(handle);

        if (claim_impedance_interfaces_)
        {
            // the stiffness, damping and set point "positions" are the corresponding state values (the set point state is the joint position)
            JointHandle stiffness, damping, set_point;
            stiffness.name = joint + std::string("_stiffness");
            stiffness.position = findInterface(state_interfaces_, joint, lwr_hw::HW_IF_STIFFNESS);
            stiffness.velocity = stiffness.effort = stiffness.position;
            stiffness.command = findInterface(command_interfaces_, joint, lwr_hw::HW_IF_STIFFNESS);
            damping.name = joint + std::string("_damping");
            damping.position = findInterface(state_interfaces_, joint, lwr_hw::HW_IF_DAMPING);
            damping.velocity = damping.effort = damping.position;
            damping.command = findInterface(command_interfaces_, joint, lwr_hw::HW_IF_DAMPING);
            set_point.name = joint + std::string("_set_point");
            set_point.position = handle.position;
            set_point.velocity = handle.velocity;
            set_point.effort = handle.effort;
            set_point.command = findInterface(command_interfaces_, joint, lwr_hw::HW_IF_SET_POINT);
            if (!stiffness.position || !stiffness.command || !damping.position || !damping.command || !set_point.command)
            {
                RCLCPP_ERROR(logger(), "Could not get the stiffness/damping/set_point interfaces of joint %s", joint.c_str());
                return false;
            }
            joint_stiffness_handles_.push_back(stiffness);
            joint_damping_handles_.push_back(damping);
            joint_set_point_handles_.push_back(set_point);
        }
    }
    return true;
}

}
