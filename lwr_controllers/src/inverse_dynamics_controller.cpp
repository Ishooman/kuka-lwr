
#include <lwr_controllers/inverse_dynamics_controller.h>

#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>

namespace lwr_controllers
{

  InverseDynamicsController::InverseDynamicsController()
    : KinematicChainControllerBase(CommandType::EFFORT)
      ,robot_description_("")
      ,root_name_("")
      ,tip_name_("")
      ,joint_names_()
      ,n_dof_(0)
      ,id_solver_(nullptr)
      ,ext_wrenches_()
      ,joint_states_()
      ,torques_()
  {}

  InverseDynamicsController::~InverseDynamicsController()
  {
    ext_wrench_sub_.reset();
  }

  controller_interface::CallbackReturn InverseDynamicsController::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    if (KinematicChainControllerBase::on_configure(previous_state) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    n_dof_ = kdl_chain_.getNrOfJoints();

    //Create inverse dynamics solver
    id_solver_.reset( new KDL::ChainIdSolver_RNE( kdl_chain_, gravity_) );

    // Resize working vectors
    joint_states_.resize(n_dof_);
    torques_.resize(n_dof_);
    ext_wrenches_.resize(kdl_chain_.getNrOfSegments());

    // Zero out torque data
    torques_.data.setZero();

    // Subscribe to external wrench topic
    ext_wrench_sub_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "~/ext_wrench", 1,
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { ext_wrench_inbox_.write(msg); });

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type InverseDynamicsController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    (void)time;
    (void)period;

    if (auto msg = ext_wrench_inbox_.take())
      ext_wrench_cb(msg);

    // Read the positions/velocities
    for(unsigned int j=0; j<n_dof_; j++) {
      joint_states_.q(j) = joint_handles_[j].getPosition();
      joint_states_.qdot(j) = joint_handles_[j].getVelocity();
      // JointStateInterface has no acceleration support
      // joint_states_.qdotdot(j) = joint_handles_[j].getAcceleration();
      joint_states_.qdotdot(j) = 0.0;
    }

    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
    if(id_solver_->CartToJnt(
            joint_states_.q,
            joint_states_.qdot,
            joint_states_.qdotdot,
            ext_wrenches_,
            torques_) != 0)
    {
      RCLCPP_ERROR(logger(), "Could not compute joint torques! Setting all torques to zero!");
      KDL::SetToZero(torques_);
    }

    // Set the commands
    for(unsigned int j=0; j<n_dof_; j++) {
      	joint_handles_[j].setCommand(torques_(j));
    	//joint_handles_[j].setCommand(0);
    }

    return controller_interface::return_type::OK;
  }

  void InverseDynamicsController::ext_wrench_cb(
      const geometry_msgs::msg::WrenchStamped::SharedPtr &wrench_msg)
  {
    // TODO: Transform wrench into appropriate frame (probably local frame for each link)

    // Convert to KDL
    KDL::Wrench wrench(
        KDL::Vector(wrench_msg->wrench.force.x,
                    wrench_msg->wrench.force.y,
                    wrench_msg->wrench.force.z),
        KDL::Vector(wrench_msg->wrench.torque.x,
                    wrench_msg->wrench.torque.y,
                    wrench_msg->wrench.torque.z));

    // Set the wrench on the tip link
    ext_wrenches_.back() = wrench;
  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::InverseDynamicsController, controller_interface::ControllerInterface)

