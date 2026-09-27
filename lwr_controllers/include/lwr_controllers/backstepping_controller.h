#ifndef LWR_CONTROLLERS__BACKSTEPPING_CONTROL_H
#define LWR_CONTROLLERS__BACKSTEPPING_CONTROL_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/msg/multi_priority_task.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <control_toolbox/pid.hpp>

#include <Eigen/Core>
#include <sstream>

namespace lwr_controllers
{

	class BacksteppingController: public controller_interface::PIDKinematicChainControllerBase
	{
	public:
		BacksteppingController();
		~BacksteppingController();

		CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
		CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
		controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
		void command(const lwr_controllers::msg::MultiPriorityTask::SharedPtr &msg);
		void set_marker(KDL::Frame x, int id);

	private:
		rclcpp::Subscription<lwr_controllers::msg::MultiPriorityTask>::SharedPtr sub_command_;
		controller_interface::CommandInbox<lwr_controllers::msg::MultiPriorityTask> command_inbox_;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_error_;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pose_;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_traj_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
		std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_pub_error_;
		std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_pub_pose_;
		std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_pub_traj_;
		std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>> rt_pub_marker_;

		std_msgs::msg::Float64MultiArray msg_err_;
		std_msgs::msg::Float64MultiArray msg_pose_;
		std_msgs::msg::Float64MultiArray msg_traj_;
		visualization_msgs::msg::Marker msg_marker_;

		KDL::JntArrayAcc joint_ref_;

		KDL::Frame x_,x0_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity

		KDL::Frame x_des_;	//desired pose
		Eigen::Matrix<double,6,1> x_des_dot_;
		Eigen::Matrix<double,6,1> x_des_dotdot_;

		Eigen::Matrix<double,6,1> x_ref_dot_;
		Eigen::Matrix<double,6,1> x_ref_dotdot_;

		KDL::Twist x_err_;

		KDL::JntArray Kp_,Kd_;

		KDL::JntArray tau_;

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))
		Eigen::MatrixXd J_pinv_;

		int step_ = 0;
		int first_step_ = 0;
		int msg_id_ = 0;
		int cmd_flag_ = 0;

		std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		std::unique_ptr<KDL::ChainDynParam> id_solver_;
		std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
	};

}

#endif
