#ifndef LWR_CONTROLLERS__INVERSE_DYNAMICS_CONTROLLER_H
#define LWR_CONTROLLERS__INVERSE_DYNAMICS_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace lwr_controllers
{

  class InverseDynamicsController : public controller_interface::KinematicChainControllerBase
  {
  public:

    InverseDynamicsController();
    ~InverseDynamicsController();

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
    {
      if (KinematicChainControllerBase::on_activate(previous_state) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;
      KDL::SetToZero(torques_);
      return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ext_wrench_sub_;
    controller_interface::CommandInbox<geometry_msgs::msg::WrenchStamped> ext_wrench_inbox_;
    void ext_wrench_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr &wrench_msg);

    std::string
      robot_description_,
      root_name_,
      tip_name_;
    std::vector<std::string> joint_names_;


    unsigned int n_dof_;
    std::unique_ptr<KDL::ChainIdSolver_RNE> id_solver_;

    KDL::Wrenches ext_wrenches_;
    KDL::JntArrayAcc joint_states_;
    KDL::JntArray torques_;

  };

}

#endif
