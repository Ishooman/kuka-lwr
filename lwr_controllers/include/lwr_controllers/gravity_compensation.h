#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace lwr_controllers
{
    class GravityCompensation: public controller_interface::KinematicChainControllerBase
    {
    public:

        GravityCompensation();
        ~GravityCompensation();

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // private:

        // std::vector<float> previous_stiffness_; /// stiffness before activating controller

        // hack required as long as there is separate position handle for stiffness
        // std::vector<hardware_interface::JointHandle> joint_stiffness_handles_;

        // const static float DEFAULT_STIFFNESS = 0.01;

    };
}

#endif
