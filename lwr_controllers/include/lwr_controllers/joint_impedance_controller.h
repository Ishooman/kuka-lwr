
#ifndef LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/*
	tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/

namespace lwr_controllers
{

	class JointImpedanceController: public controller_interface::KinematicChainControllerBase
	{
	public:

		JointImpedanceController();
		~JointImpedanceController();

		CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
		CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

		controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
		void command(const std_msgs::msg::Float64MultiArray::SharedPtr &msg);
		void setParam(const std_msgs::msg::Float64MultiArray::SharedPtr &msg, KDL::JntArray* array, std::string s);

	private:

		using MsgType = std_msgs::msg::Float64MultiArray;

		rclcpp::Subscription<MsgType>::SharedPtr sub_stiffness_, sub_damping_, sub_add_torque_;
		rclcpp::Subscription<MsgType>::SharedPtr sub_posture_;
		controller_interface::CommandInbox<MsgType> stiffness_inbox_, damping_inbox_, add_torque_inbox_, posture_inbox_;

		KDL::JntArray q_des_;
		KDL::JntArray tau_des_;
		KDL::JntArray K_, D_;

		// gains from the yaml file, NaN if not set
		double stiffness_gains_, damping_gains_;

	};

} // namespace

#endif
