#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/joint_impedance_controller.h>

namespace lwr_controllers {

JointImpedanceController::JointImpedanceController() : KinematicChainControllerBase(CommandType::EFFORT, true) {}

JointImpedanceController::~JointImpedanceController() {}

controller_interface::CallbackReturn JointImpedanceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    if (KinematicChainControllerBase::on_configure(previous_state) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());

    // the current values of the hardware are used for the gains not set in the yaml file (see on_activate)
    stiffness_gains_ = getParamDouble("stiffness_gains", std::numeric_limits<double>::quiet_NaN());
    damping_gains_ = getParamDouble("damping_gains", std::numeric_limits<double>::quiet_NaN());

    sub_stiffness_ = get_node()->create_subscription<MsgType>("~/stiffness", 1, [this](const MsgType::SharedPtr msg) { stiffness_inbox_.write(msg); });
    sub_damping_ = get_node()->create_subscription<MsgType>("~/damping", 1, [this](const MsgType::SharedPtr msg) { damping_inbox_.write(msg); });
    sub_add_torque_ = get_node()->create_subscription<MsgType>("~/additional_torque", 1, [this](const MsgType::SharedPtr msg) { add_torque_inbox_.write(msg); });
    sub_posture_ = get_node()->create_subscription<MsgType>("~/command", 1, [this](const MsgType::SharedPtr msg) { posture_inbox_.write(msg); });

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    if (KinematicChainControllerBase::on_activate(previous_state) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        K_(i) = joint_stiffness_handles_[i].getPosition();
        D_(i) = joint_damping_handles_[i].getPosition();
    }

    RCLCPP_DEBUG(logger(), " Number of joints in handle = %lu", joint_handles_.size() );

    for (size_t i = 0; i < joint_handles_.size(); ++i) {
        if ( std::isnan(stiffness_gains_) ) {
            RCLCPP_WARN(logger(), "Stiffness gain not set in yaml file, Using %f", K_(i));
        }
        else {
            K_(i) = stiffness_gains_;
        }
    }
    for (size_t i = 0; i < joint_handles_.size(); ++i) {
        if ( std::isnan(damping_gains_) ) {
            RCLCPP_WARN(logger(), "Damping gain not set in yaml file, Using %f", D_(i));
        }
        else {
            D_(i) = damping_gains_;
        }
    }

    // Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
        tau_des_(i) = 0.0;
        q_des_(i) = joint_handles_[i].getPosition();
    }

    // discard the messages received while inactive
    stiffness_inbox_.reset();
    damping_inbox_.reset();
    add_torque_inbox_.reset();
    posture_inbox_.reset();

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    (void)time;
    (void)period;

    // process the new commands
    if (auto msg = stiffness_inbox_.take())
        setParam(msg, &K_, "K");
    if (auto msg = damping_inbox_.take())
        setParam(msg, &D_, "D");
    if (auto msg = add_torque_inbox_.take())
        setParam(msg, &tau_des_, "AddTorque");
    if (auto msg = posture_inbox_.take())
        command(msg);

    //Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(tau_des_(i));
        joint_stiffness_handles_[i].setCommand(K_(i));
        joint_damping_handles_[i].setCommand(D_(i));
        joint_set_point_handles_[i].setCommand(q_des_(i));
    }

    return controller_interface::return_type::OK;
}


void JointImpedanceController::command(const std_msgs::msg::Float64MultiArray::SharedPtr &msg) {
    if (msg->data.size() == 0) {
        RCLCPP_INFO(logger(), "Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if (msg->data.size() != joint_handles_.size()) {
        RCLCPP_ERROR(logger(), "Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        for (unsigned int j = 0; j < joint_handles_.size(); ++j)
            q_des_(j) = msg->data[j];
    }

}

void JointImpedanceController::setParam(const std_msgs::msg::Float64MultiArray::SharedPtr &msg, KDL::JntArray* array, std::string s)
{
    if (msg->data.size() == joint_handles_.size())
    {
        for (unsigned int i = 0; i < joint_handles_.size(); ++i)
        {
            (*array)(i) = msg->data[i];
        }
    }
    else
    {
        RCLCPP_INFO(logger(), "Num of Joint handles = %lu", joint_handles_.size());
    }

    RCLCPP_INFO(logger(), "Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

    RCLCPP_INFO(logger(), "New param %s: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", s.c_str(),
             (*array)(0), (*array)(1), (*array)(2), (*array)(3), (*array)(4), (*array)(5), (*array)(6));
}

} // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointImpedanceController, controller_interface::ControllerInterface)
