#include <pluginlib/class_list_macros.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <lwr_controllers/computed_torque_controller.h>

namespace lwr_controllers
{
	ComputedTorqueController::ComputedTorqueController() : KinematicChainControllerBase(CommandType::EFFORT) {}
	ComputedTorqueController::~ComputedTorqueController() {}

	controller_interface::CallbackReturn ComputedTorqueController::on_configure(const rclcpp_lifecycle::State & previous_state)
	{
        if (KinematicChainControllerBase::on_configure(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

		id_solver_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

		cmd_states_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kv_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
        joint_initial_states_.resize(kdl_chain_.getNrOfJoints());
        current_cmd_.resize(kdl_chain_.getNrOfJoints());

		sub_posture_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("~/command", 1,
			[this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { posture_inbox_.write(msg); });
		sub_gains_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("~/set_gains", 1,
			[this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { gains_inbox_.write(msg); });

		return CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn ComputedTorqueController::on_activate(const rclcpp_lifecycle::State & previous_state)
	{
        if (KinematicChainControllerBase::on_activate(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

  		// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++)
  		{

  			Kp_(i) = 300.0;
  			Kv_(i) = 0.7;
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    	}

    	lambda = 0.1;	// lower values: flatter
    	cmd_flag_ = 0;
    	step_ = 0;

    	posture_inbox_.reset();

    	RCLCPP_INFO(logger(), " Number of joints in handle = %lu", joint_handles_.size() );

    	return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ComputedTorqueController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        (void)time;
        (void)period;

        // process the new commands
        if (auto msg = posture_inbox_.take())
            command(msg);
        if (auto msg = gains_inbox_.take())
            set_gains(msg);

    	// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++)
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    	}

    	if(cmd_flag_)
    	{
            if(step_ == 0)
            {
                joint_initial_states_ = joint_msr_states_.q;
            }
            // reaching desired joint position using a hyperbolic tangent function
            double th = tanh(M_PI-lambda*step_);
            double ch = cosh(M_PI-lambda*step_);
            double sh2 = 1.0/(ch*ch);

            for(size_t i=0; i<joint_handles_.size(); i++)
            {
                // TODO: take into account also initial/final velocity and acceleration
                current_cmd_(i) = cmd_states_(i) - joint_initial_states_(i);
                joint_des_states_.q(i) = current_cmd_(i)*0.5*(1.0-th) + joint_initial_states_(i);
                joint_des_states_.qdot(i) = current_cmd_(i)*0.5*lambda*sh2;
                joint_des_states_.qdotdot(i) = current_cmd_(i)*lambda*lambda*sh2*th;
            }
            ++step_;

    		if(joint_des_states_.q == cmd_states_)
    		{
    			cmd_flag_ = 0;	//reset command flag
    			step_ = 0;
    			RCLCPP_INFO(logger(), "Posture OK");
    		}
    	}

    	// computing Inertia, Coriolis and Gravity matrices
    	id_solver_->JntToMass(joint_msr_states_.q, M_);
    	id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    	id_solver_->JntToGravity(joint_msr_states_.q, G_);

        // PID controller
        KDL::JntArray pid_cmd_(joint_handles_.size());
        // compensation of Coriolis and Gravity
        KDL::JntArray cg_cmd_(joint_handles_.size());
        for(size_t i=0; i<joint_handles_.size(); i++)
        {
            // control law
            pid_cmd_(i) = joint_des_states_.qdotdot(i) + Kv_(i)*(joint_des_states_.qdot(i) - joint_msr_states_.qdot(i)) + Kp_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i));
            cg_cmd_(i) = C_(i) + G_(i);
        }
        tau_cmd_.data = M_.data * pid_cmd_.data;
        KDL::Add(tau_cmd_,cg_cmd_,tau_cmd_);

        for(size_t i=0; i<joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(tau_cmd_(i));
        }

        return controller_interface::return_type::OK;
    }

    void ComputedTorqueController::command(const std_msgs::msg::Float64MultiArray::SharedPtr &msg)
    {
    	if(msg->data.size() == 0)
    		RCLCPP_INFO(logger(), "Desired configuration must be of dimension %lu", joint_handles_.size());
    	else if(msg->data.size() != joint_handles_.size())
    	{
    		RCLCPP_ERROR(logger(), "Posture message had the wrong size: %u", (unsigned int)msg->data.size());
    		return;
    	}
    	else
    	{
    		for (unsigned int i = 0; i<joint_handles_.size(); i++)
    			cmd_states_(i) = msg->data[i];

    		cmd_flag_ = 1;
            // when a new command is set, steps should be reset to avoid jumps in the update
            step_ = 0;
    	}

	}

	void ComputedTorqueController::set_gains(const std_msgs::msg::Float64MultiArray::SharedPtr &msg)
	{
		if(msg->data.size() == 2*joint_handles_.size())
		{
			for(unsigned int i = 0; i < joint_handles_.size(); i++)
			{
				Kp_(i) = msg->data[i];
				Kv_(i) = msg->data[i + joint_handles_.size()];
			}
		}
		else
			RCLCPP_INFO(logger(), "Number of Joint handles = %lu", joint_handles_.size());

		RCLCPP_INFO(logger(), "Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

		RCLCPP_INFO(logger(), "New gains Kp: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5), Kp_(6));
		RCLCPP_INFO(logger(), "New gains Kv: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kv_(0), Kv_(1), Kv_(2), Kv_(3), Kv_(4), Kv_(5), Kv_(6));

	}
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::ComputedTorqueController, controller_interface::ControllerInterface)
