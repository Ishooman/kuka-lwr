#ifndef LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_DYNAMICS_H
#define LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_DYNAMICS_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/msg/multi_priority_task.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>
#include <sstream>

namespace lwr_controllers
{
	class MultiTaskPriorityInverseDynamics: public controller_interface::PIDKinematicChainControllerBase
	{
	public:
		MultiTaskPriorityInverseDynamics();
		~MultiTaskPriorityInverseDynamics();

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

		KDL::JntArray qdot_last_;

		KDL::Frame x_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity
		KDL::Twist x_dot_dot_;	//current e-e acceleration
		std::vector<KDL::Frame> x_des_;	//desired pose

		KDL::Twist x_err_;	// position error

		KDL::JntArray Kp_,Kd_;	// velocity error, position error

		KDL::JntArray tau_;	// control torque

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		std::vector<KDL::Jacobian> J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))

		Eigen::MatrixXd J_pinv_;	// Jacobian pseudo-inv

		Eigen::Matrix<double,6,1> e_ref_;	// reference error
		Eigen::Matrix<double,7,7> I_;		// Identity
		Eigen::Matrix<double,7,7> N_trans_;	// null-space matrix
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;
		Eigen::MatrixXd lambda_;
		Eigen::Matrix<double,6,1> b_;

		int first_step_ = 0;	// first step flag
		int msg_id_ = 0;		// marker message id
		int cmd_flag_ = 0;		// command received flag
		int ntasks_ = 0;		// # of task
		std::vector<bool> on_target_flag_;
		std::vector<int> links_index_;


		std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		std::unique_ptr<KDL::ChainDynParam> id_solver_;
		std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
	};

}

#endif
