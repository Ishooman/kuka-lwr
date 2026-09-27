#include <pluginlib/class_list_macros.hpp>
#include <math.h>

#include <lwr_controllers/gravity_compensation.h>

namespace lwr_controllers
{
    GravityCompensation::GravityCompensation() : KinematicChainControllerBase(CommandType::EFFORT) {}
    GravityCompensation::~GravityCompensation() {}

    controller_interface::CallbackReturn GravityCompensation::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        if (KinematicChainControllerBase::on_configure(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        // find stiffness (dummy) joints; this is necessary until proper position/stiffness/damping interface exists
        // for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        // {
        //     joint_stiffness_handles_.push_back(robot->getHandle(it->getJoint().getName()+"_stiffness"));
        // }

        // ROS_DEBUG("found %lu stiffness handles", joint_stiffness_handles_.size());

        // previous_stiffness_.resize(joint_stiffness_handles_.size());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensation::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        if (KinematicChainControllerBase::on_activate(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        // for(size_t i=0; i<joint_handles_.size(); i++)
        // {
        //    previous_stiffness_[i] = joint_stiffness_handles_[i].getPosition();
        // }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type GravityCompensation::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        (void)time;
        (void)period;

        // update the commanded position to the actual, so that the robot doesn't
        // go back at full speed to the last commanded position when the stiffness
        // is raised again
        for(size_t i=0; i<joint_handles_.size(); i++)
        {
            //joint_handles_[i].setCommand(joint_handles_[i].getPosition());
            joint_handles_[i].setCommand( 0.0 );
            // joint_stiffness_handles_[i].setCommand(DEFAULT_STIFFNESS);
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn GravityCompensation::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        //for(size_t i=0; i<joint_handles_.size(); i++)
        //{
        //    joint_stiffness_handles_[i].setCommand(previous_stiffness_[i]);
        //}
        return KinematicChainControllerBase::on_deactivate(previous_state);
    }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::GravityCompensation, controller_interface::ControllerInterface)
