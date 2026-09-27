#ifndef LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_KINEMATICS_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/msg/multi_priority_task.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>
#include <sstream>

namespace lwr_controllers
{
	class MultiTaskPriorityInverseKinematics: public controller_interface::PIDKinematicChainControllerBase
	{
	public:
		MultiTaskPriorityInverseKinematics();
		~MultiTaskPriorityInverseKinematics();

		CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
		CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
		controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
		void command(const lwr_controllers::msg::MultiPriorityTask::SharedPtr &msg);
		void set_marker(KDL::Frame x, int index, int id);

	private:
		rclcpp::Subscription<lwr_controllers::msg::MultiPriorityTask>::SharedPtr sub_command_;
		controller_interface::CommandInbox<lwr_controllers::msg::MultiPriorityTask> command_inbox_;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_error_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
		std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_pub_error_;
		std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>> rt_pub_marker_;

		std_msgs::msg::Float64MultiArray msg_err_;
		visualization_msgs::msg::MarkerArray msg_marker_;
		std::stringstream sstr_;

		KDL::Frame x_;		//current pose
		std::vector<KDL::Frame> x_des_;	//desired pose

		KDL::Twist x_err_;

		KDL::JntArray tau_cmd_;

		KDL::Jacobian J_;	//Jacobian
		KDL::Jacobian J_star_; // it will be J_*P_

		Eigen::MatrixXd J_pinv_;//<double,7,6> J_pinv_;

		Eigen::Matrix<double,6,1> e_dot_;
		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> P_;

		int msg_id_ = 0;
		int cmd_flag_ = 0;
		int ntasks_ = 0;
		std::vector<bool> on_target_flag_;
		std::vector<int> links_index_;


		std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		std::unique_ptr<KDL::ChainDynParam> id_solver_;
		std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
	};

}

#endif
