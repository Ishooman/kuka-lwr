#include <pluginlib/class_list_macros.hpp>
#include <math.h>

#include <lwr_controllers/arm_state_controller.h>
#include <utils/kdl_msg_conversions.h>
#include <utils/pseudo_inversion.h>
#include <control_toolbox/filters.hpp>

namespace arm_state_controller
{
    ArmStateController::ArmStateController() : KinematicChainControllerBase(CommandType::NONE) {}
    ArmStateController::~ArmStateController() {}

    controller_interface::CallbackReturn ArmStateController::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        if (KinematicChainControllerBase::on_configure(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        // get publishing period
        publish_rate_ = getParamDouble("publish_rate", std::numeric_limits<double>::quiet_NaN());
        if (std::isnan(publish_rate_)) {
            RCLCPP_ERROR(logger(), "Parameter 'publish_rate' not set");
            return CallbackReturn::ERROR;
        }

        pub_ = get_node()->create_publisher<lwr_controllers::msg::ArmState>("~/arm_state", 4);
        realtime_pub_.reset(new realtime_tools::RealtimePublisher<lwr_controllers::msg::ArmState>(pub_));
        realtime_pub_->msg_.est_ext_torques.resize(kdl_chain_.getNrOfJoints());

        gravity_.reset(new KDL::Vector(0.0, 0.0, -9.81)); // TODO: compute from actual robot position (TF?)
        id_solver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_, *gravity_));
        jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        jacobian_.reset(new KDL::Jacobian(kdl_chain_.getNrOfJoints()));
        joint_position_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_velocity_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_acceleration_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_wrenches_.reset(new KDL::Wrenches(kdl_chain_.getNrOfSegments()));
        joint_effort_est_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ArmStateController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        if (KinematicChainControllerBase::on_activate(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        last_publish_time_ = get_node()->now();
        realtime_pub_->msg_.joint_name.clear();
        for (unsigned i = 0; i < joint_handles_.size(); i++){
            (*joint_position_)(i) = joint_handles_[i].getPosition();
            (*joint_velocity_)(i) = joint_handles_[i].getVelocity();
            (*joint_acceleration_)(i) = 0;
            realtime_pub_->msg_.joint_name.push_back(joint_handles_[i].getName());
        }
        RCLCPP_INFO(logger(), "started");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ArmStateController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        if (last_publish_time_.get_clock_type() != time.get_clock_type())
            last_publish_time_ = time;

        // limit rate of publishing
        if (publish_rate_ > 0.0 && last_publish_time_ + rclcpp::Duration::from_seconds(1.0/publish_rate_) < time){

            if (realtime_pub_->trylock()) {
                // we're actually publishing, so increment time
                last_publish_time_ = last_publish_time_ + rclcpp::Duration::from_seconds(1.0/publish_rate_);

                for (unsigned i = 0; i < joint_handles_.size(); i++) {
                    float acceleration = (period.seconds() > 0.0) ? filters::exponentialSmoothing((joint_handles_[i].getVelocity() - (*joint_velocity_)(i))/period.seconds(), (*joint_acceleration_)(i), 0.2) : 0.0;
                    (*joint_position_)(i) = joint_handles_[i].getPosition();
                    (*joint_velocity_)(i) = joint_handles_[i].getVelocity();
                    (*joint_acceleration_)(i) = acceleration;
                }

                // Compute Dynamics
                int ret = id_solver_->CartToJnt(*joint_position_,
                                                *joint_velocity_,
                                                *joint_acceleration_,
                                                *joint_wrenches_,
                                                *joint_effort_est_);
                if (ret < 0) {
                    RCLCPP_ERROR(logger(), "KDL: inverse dynamics ERROR");
                    realtime_pub_->unlock();
                    return controller_interface::return_type::OK;
                }

                realtime_pub_->msg_.header.stamp = time;
                for (unsigned i=0; i<joint_handles_.size(); i++) {
                    realtime_pub_->msg_.est_ext_torques[i] = joint_handles_[i].getEffort() - (*joint_effort_est_)(i);
                }

                // Compute cartesian wrench on end effector
                ret = jac_solver_->JntToJac(*joint_position_, *jacobian_);
                if (ret < 0) {
                    RCLCPP_ERROR(logger(), "KDL: jacobian computation ERROR");
                    realtime_pub_->unlock();
                    return controller_interface::return_type::OK;
                }

                Eigen::MatrixXd jinv;
                pseudo_inverse(jacobian_->data.transpose(),jinv,true);

                KDL::Wrench wrench;

                for (unsigned int i = 0; i < 6; i++)
                    for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); j++)
                        wrench[i] += jinv(i,j) * realtime_pub_->msg_.est_ext_torques[j];

                tf::wrenchKDLToMsg(wrench, realtime_pub_->msg_.est_ee_wrench_base);

                // Transform cartesian wrench into tool reference frame
                KDL::Frame tool_frame;
                fk_solver_->JntToCart(*joint_position_, tool_frame);
                KDL::Wrench tool_wrench = tool_frame * wrench;

                tf::wrenchKDLToMsg(tool_wrench, realtime_pub_->msg_.est_ee_wrench);

                realtime_pub_->unlockAndPublish();
            }
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn ArmStateController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        return KinematicChainControllerBase::on_deactivate(previous_state);
    }

}

PLUGINLIB_EXPORT_CLASS(arm_state_controller::ArmStateController, controller_interface::ControllerInterface)
