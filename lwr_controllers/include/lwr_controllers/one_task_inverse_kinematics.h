#ifndef LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H

#include "KinematicChainControllerBase.h"
#include "lwr_controllers/msg/pose_rpy.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <Eigen/Core>
#include <sstream>

namespace lwr_controllers
{
	class OneTaskInverseKinematics: public controller_interface::KinematicChainControllerBase
	{
	public:
		OneTaskInverseKinematics();
		~OneTaskInverseKinematics();

		CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
		CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
		controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
		void command(const lwr_controllers::msg::PoseRPY::SharedPtr &msg);

	private:
		rclcpp::Subscription<lwr_controllers::msg::PoseRPY>::SharedPtr sub_command_;
		controller_interface::CommandInbox<lwr_controllers::msg::PoseRPY> command_inbox_;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_;	//desired pose

		KDL::Twist x_err_;

		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;
		Eigen::Matrix<double,3,3> skew_;

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;

		int cmd_flag_;

		std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
	};

}

#endif
