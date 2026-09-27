#ifndef ARM_STATE_CONTROLLER_H
#define ARM_STATE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <lwr_controllers/msg/arm_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <realtime_tools/realtime_publisher.hpp>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace arm_state_controller
{
    class ArmStateController: public controller_interface::KinematicChainControllerBase
    {
    public:

        ArmStateController();
        ~ArmStateController();

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    private:

        rclcpp::Publisher<lwr_controllers::msg::ArmState>::SharedPtr pub_;
        std::shared_ptr< realtime_tools::RealtimePublisher< lwr_controllers::msg::ArmState > > realtime_pub_;

        std::unique_ptr<KDL::ChainIdSolver_RNE> id_solver_;
        std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        std::unique_ptr<KDL::ChainFkSolverPos> fk_solver_;
        std::unique_ptr<KDL::Jacobian> jacobian_;
        std::unique_ptr<KDL::Vector> gravity_;
        std::unique_ptr<KDL::JntArray> joint_position_;
        std::unique_ptr<KDL::JntArray> joint_velocity_;
        std::unique_ptr<KDL::JntArray> joint_acceleration_;
        std::unique_ptr<KDL::Wrenches> joint_wrenches_;
        std::unique_ptr<KDL::JntArray> joint_effort_est_;

        rclcpp::Time last_publish_time_;
        double publish_rate_;
    };
}

#endif
