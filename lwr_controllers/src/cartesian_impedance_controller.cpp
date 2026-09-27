#include <math.h>
#include <pluginlib/class_list_macros.hpp>
#include <utils/kdl_msg_conversions.h>

#include "lwr_controllers/cartesian_impedance_controller.h"

namespace lwr_controllers
{
    CartesianImpedanceController::CartesianImpedanceController() : publish_cartesian_pose_(false), cur_T_FRI_(12) {}
    CartesianImpedanceController::~CartesianImpedanceController() {}

    controller_interface::CallbackReturn CartesianImpedanceController::on_init()
    {
        try
        {
            auto_declare<std::string>("robot_name", "");
            auto_declare<std::string>("root_name", "");
            auto_declare<std::string>("tip_name", "");
            auto_declare<bool>("publish_cartesian_pose", false);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    std::vector<std::string> CartesianImpedanceController::commandInterfaceNames() const
    {
        // 12 for cart pos, 6 for stiff, 6 for damp, 6 for wrench
        std::vector<std::string> names;
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                names.push_back(cart_prefix_ + "/" + cart_12_names_.at(c));
            if(c > 11 && c < 18)
                names.push_back(cart_prefix_ + "/" + cart_6_names_.at(c-12) + std::string("_stiffness"));
            if(c > 17 && c < 24)
                names.push_back(cart_prefix_ + "/" + cart_6_names_.at(c-18) + std::string("_damping"));
            if(c > 23 && c < 30)
                names.push_back(cart_prefix_ + "/" + cart_6_names_.at(c-24) + std::string("_wrench"));
        }
        return names;
    }

    controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const
    {
        return {controller_interface::interface_configuration_type::INDIVIDUAL, commandInterfaceNames()};
    }

    controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto & name : cart_12_names_)
            config.names.push_back(cart_prefix_ + "/" + name);
        return config;
    }

    controller_interface::CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        auto n = get_node();
        RCLCPP_INFO(n->get_logger(), "THIS CONTROLLER USES THE TCP INFORMATION SET ON THE FRI SIDE... SO BE SURE YOU KNOW WHAT YOU ARE DOING!");

        robot_namespace_ = n->get_parameter("robot_name").as_string();
        if (robot_namespace_.empty())
        {
            robot_namespace_ = n->get_namespace();
            robot_namespace_.erase(0, robot_namespace_.find_first_not_of('/'));
            RCLCPP_WARN_STREAM(n->get_logger(), "CartesianImpedanceController: Could not read robot name from parameter server ("<<n->get_namespace() << "/" << n->get_name()<<"/robot_name), using the namespace (" << robot_namespace_ << ")...");
        }
        cart_prefix_ = robot_namespace_ + std::string("_cart");

        root_name_ = n->get_parameter("root_name").as_string();
        robotBase_controllerBase_ = KDL::Frame::Identity();
        if (root_name_.empty())
        {
            RCLCPP_WARN_STREAM(n->get_logger(), "CartesianImpedanceController: Could not read robot root name from parameter server ("<<n->get_namespace() << "/" << n->get_name()<<"/root_name). Considering it to be equal to #BASE as defined in the KRC!!!");
            root_name_ = robot_namespace_ + "_base_link";
        }
        else if (root_name_ != robot_namespace_ + "_base_link")
        {
            tf2_ros::Buffer buffer(n->get_clock());
            tf2_ros::TransformListener listener(buffer, n, true);
            geometry_msgs::msg::TransformStamped transform;
            try{
                transform = buffer.lookupTransform( robot_namespace_ + "_base_link", root_name_, tf2::TimePointZero, tf2::durationFromSec(1.0));
            }
            catch (const tf2::TransformException & ex){
                RCLCPP_ERROR_STREAM(n->get_logger(), "CartesianImpedanceController: Could not read transform from "<< robot_namespace_ << "_base_link to " << root_name_ << ": " << ex.what());
                return CallbackReturn::ERROR;
            }

            tf::transformMsgToKDL(transform, robotBase_controllerBase_);
        }
        tip_name_ = n->get_parameter("tip_name").as_string();
        if (!tip_name_.empty())
        {
            RCLCPP_WARN_STREAM(n->get_logger(), "CartesianImpedanceController: robot tip name is " << tip_name_ << ". In this controller it is ignored. Using tip name defined in #TOOL in the KRC!!!");
        }
        publish_cartesian_pose_ = n->get_parameter("publish_cartesian_pose").as_bool();
        if (publish_cartesian_pose_)
        {
            RCLCPP_WARN_STREAM(n->get_logger(), "Publishing the cartesian position of the robot tip (the #TOOL defined in the KRC!!!)");
        }

        cart_12_names_ = {"rot_xx", "rot_yx", "rot_zx", "pos_x",
                          "rot_xy", "rot_yy", "rot_zy", "pos_y",
                          "rot_xz", "rot_yz", "rot_zz", "pos_z"};
        cart_6_names_ = {"X", "Y", "Z", "A", "B", "C"};

        sub_command_ = n->create_subscription<lwr_controllers::msg::CartesianImpedancePoint>("~/command", 1,
            [this](const lwr_controllers::msg::CartesianImpedancePoint::SharedPtr msg) { command_inbox_.write(msg); });
        srv_command_ = n->create_service<lwr_controllers::srv::SetCartesianImpedanceCommand>("~/set_command",
            std::bind(&CartesianImpedanceController::command_cb, this, std::placeholders::_1, std::placeholders::_2));
        sub_ft_measures_ = n->create_subscription<geometry_msgs::msg::WrenchStamped>("~/ft_measures", 1,
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { ft_inbox_.write(msg); });
        pub_goal_ = n->create_publisher<geometry_msgs::msg::PoseStamped>("~/goal", 1);
        realtime_goal_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>(pub_goal_));
        if (publish_cartesian_pose_)
        {
            pub_pose_ = n->create_publisher<geometry_msgs::msg::PoseStamped>("~/cartesian_pose", 4);
            realtime_pose_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>(pub_pose_));
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;

        // now get all handles, 12 for cart pos, 6 for stiff, 6 for damp, 6 for wrench
        cart_handles_.clear();
        for (const auto & name : commandInterfaceNames())
        {
            hardware_interface::LoanedCommandInterface* handle = nullptr;
            for (auto & interface : command_interfaces_)
                if (interface.get_name() == name)
                    handle = &interface;
            if (!handle)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not get the command interface %s", name.c_str());
                return CallbackReturn::ERROR;
            }
            cart_handles_.push_back(handle);
        }
        cart_state_handles_.clear();
        for (const auto & name : cart_12_names_)
        {
            const hardware_interface::LoanedStateInterface* handle = nullptr;
            for (const auto & interface : state_interfaces_)
                if (interface.get_name() == cart_prefix_ + "/" + name)
                    handle = &interface;
            if (!handle)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not get the state interface %s/%s", cart_prefix_.c_str(), name.c_str());
                return CallbackReturn::ERROR;
            }
            cart_state_handles_.push_back(handle);
        }

        getCurrentPose(x_ref_);
        x_des_ = x_ref_;

        // Initial Cartesian stiffness
        KDL::Stiffness k( 800.0, 800.0, 800.0, 50.0, 50.0, 50.0 );
        k_des_ = k;

        // Initial Cartesian damping
        KDL::Stiffness d( 0.8, 0.8, 0.8, 0.8, 0.8, 0.8 );
        d_des_ = d;

        // Initial force/torque measure
        KDL::Wrench w(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
        f_des_ = w;

        command_inbox_.reset();

        // forward initial commands to hwi
        forwardCmdFRI(x_des_);

        return CallbackReturn::SUCCESS;
    }

    void CartesianImpedanceController::command(const lwr_controllers::msg::CartesianImpedancePoint::SharedPtr &msg)
    {
        // Validate command, for now, only check non-zero of stiffness, damping, and orientation


        // Compute a KDL frame out of the message
        tf::poseMsgToKDL( msg->x_fri, x_des_ );
        // Pre-multiply the pose to make sure it is expressed in the correct frame
        x_des_ = robotBase_controllerBase_ * x_des_;

        // Convert Wrench msg to KDL wrench
        tf::wrenchMsgToKDL( msg->f_fri, f_des_ );

        // Convert from Stiffness msg array to KDL stiffness
        //if(!(msg->k_fri.x + msg->k_fri.y + msg->k_fri.z + msg->k_fri.rx + msg->k_fri.ry + msg->k_fri.rz == 0.0))
        //{
            RCLCPP_INFO(get_node()->get_logger(), "Updating Stiffness command");
            KDL::Stiffness k( msg->k_fri.x, msg->k_fri.y, msg->k_fri.z, msg->k_fri.rx, msg->k_fri.ry, msg->k_fri.rz );
            k_des_ = k;

            RCLCPP_INFO(get_node()->get_logger(), "Updating Damping command");
            KDL::Stiffness d( msg->d_fri.x, msg->d_fri.y, msg->d_fri.z, msg->d_fri.rx, msg->d_fri.ry, msg->d_fri.rz );
            d_des_ = d;
        //}

        // publishe goal
        geometry_msgs::msg::PoseStamped goal;
        goal.header = msg->header;
        goal.pose = msg->x_fri;
        if (realtime_goal_pub_->trylock())
        {
            realtime_goal_pub_->msg_ = goal;
            realtime_goal_pub_->unlockAndPublish();
        }

    }

    void CartesianImpedanceController::command_cb(const std::shared_ptr<lwr_controllers::srv::SetCartesianImpedanceCommand::Request> req, std::shared_ptr<lwr_controllers::srv::SetCartesianImpedanceCommand::Response> res)
    {
        (void)req;
        (void)res;
        return;
    }

    void CartesianImpedanceController::updateFT(const geometry_msgs::msg::WrenchStamped::SharedPtr &msg)
    {
        // Convert Wrench msg to KDL wrench
        geometry_msgs::msg::Wrench f_meas = msg->wrench;
        tf::wrenchMsgToKDL( f_meas, f_cur_);
    }

    controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        (void)time;
        (void)period;

        // process the new commands
        if (auto msg = command_inbox_.take())
            command(msg);
        if (auto msg = ft_inbox_.take())
            updateFT(msg);

        // get current values
        // std::cout << "Update current values" << std::endl;
        getCurrentPose(x_cur_);

        if (publish_cartesian_pose_)
        {
            publishCurrentPose(x_cur_);
        }

        // forward commands to hwi
        // std::cout << "Before forwarding the command" << std::endl;
        forwardCmdFRI(x_des_);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        cart_handles_.clear();
        cart_state_handles_.clear();
        return CallbackReturn::SUCCESS;
    }

    void CartesianImpedanceController::multiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest)
    {
        Eigen::Matrix<double,6,1> w;
        w(0) = src.force(0);
        w(1) = src.force(1);
        w(2) = src.force(2);
        w(3) = src.torque(0);
        w(4) = src.torque(1);
        w(5) = src.torque(2);

        Eigen::MatrixXd j(jac.rows(), jac.columns());
        j = jac.data;
        j.transposeInPlace();

        Eigen::VectorXd t(jac.columns());
        t = j*w;

        dest.resize(jac.columns());
        for (unsigned i=0; i<jac.columns(); i++)
        dest(i) = t(i);
    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Frame& in, std::vector<double>& out)
    {
        assert(out.size() == 12);
        out[0] = in.M.UnitX().x();
        out[1] = in.M.UnitY().x();
        out[2] = in.M.UnitZ().x();
        out[3] = in.p.x();
        out[4] = in.M.UnitX().y();
        out[5] = in.M.UnitY().y();
        out[6] = in.M.UnitZ().y();
        out[7] = in.p.y();
        out[8] = in.M.UnitX().z();
        out[9] = in.M.UnitY().z();
        out[10] = in.M.UnitZ().z();
        out[11] = in.p.z();

    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Stiffness& in, std::vector<double>& out)
    {
        assert(out.size() == 6);
        for( int d = 0; d < 6; ++d)
        {
            out[d] = in[d];
        }
    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Wrench& in, std::vector<double>& out)
    {
        assert(out.size() == 6);
        out[0] = in.force.x();
        out[1] = in.force.y();
        out[2] = in.force.z();
        out[3] = in.torque.x();
        out[4] = in.torque.y();
        out[5] = in.torque.z();
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out)
    {
        assert(in.size() == 12);
        KDL::Rotation R(in[0], in[1], in[2], in[4], in[5], in[6], in[8], in[9], in[10]);
        KDL::Vector p(in[3], in[7], in[11]);
        KDL::Frame T(R, p);
        out = T;
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out)
    {
        assert(in.size() == 6);
        KDL::Stiffness s(in[0], in[1], in[2], in[3], in[4], in[5]);
        out = s;
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out)
    {
        assert(in.size() == 6);
        KDL::Wrench w(KDL::Vector(in[0], in[1], in[2]), KDL::Vector(in[3], in[4], in[5]));
        out = w;
    }

    void CartesianImpedanceController::forwardCmdFRI(const KDL::Frame& f)
    {
        fromKDLtoFRI(f, cur_T_FRI_);
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.at(c)->set_value(cur_T_FRI_[c]);
            else if(c < 18)
                cart_handles_.at(c)->set_value(k_des_[c-12]);
            else if(c < 24)
                cart_handles_.at(c)->set_value(d_des_[c-18]);
            else if(c < 30)
                cart_handles_.at(c)->set_value(f_des_[c-24]);
        }
    }

    void CartesianImpedanceController::getCurrentPose(KDL::Frame& f)
    {
        KDL::Rotation cur_R(cart_state_handles_.at(0)->get_value(),
                            cart_state_handles_.at(1)->get_value(),
                            cart_state_handles_.at(2)->get_value(),
                            cart_state_handles_.at(4)->get_value(),
                            cart_state_handles_.at(5)->get_value(),
                            cart_state_handles_.at(6)->get_value(),
                            cart_state_handles_.at(8)->get_value(),
                            cart_state_handles_.at(9)->get_value(),
                            cart_state_handles_.at(10)->get_value());
        KDL::Vector cur_p(cart_state_handles_.at(3)->get_value(),
                          cart_state_handles_.at(7)->get_value(),
                          cart_state_handles_.at(11)->get_value());
        f = KDL::Frame( cur_R, cur_p );
    }

    void CartesianImpedanceController::publishCurrentPose(const KDL::Frame& f)
    {
        if (realtime_pose_pub_->trylock()) {
            realtime_pose_pub_->msg_.header.stamp = get_node()->now();
            tf::poseKDLToMsg(f, realtime_pose_pub_->msg_.pose);
            realtime_pose_pub_->unlockAndPublish();
        }
    }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
