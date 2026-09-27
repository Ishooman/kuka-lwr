#ifndef PID_KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define PID_KINEMATIC_CHAIN_CONTROLLER_BASE_H

#include "KinematicChainControllerBase.h"

#include <control_toolbox/pid.hpp>

namespace controller_interface
{
	class PIDKinematicChainControllerBase: public KinematicChainControllerBase
	{

	public:
		PIDKinematicChainControllerBase() : KinematicChainControllerBase(CommandType::EFFORT) {}
		~PIDKinematicChainControllerBase() {}

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

	protected:
        std::vector<control_toolbox::Pid> PIDs_;
        double Kp,Ki,Kd;

        // helper to call computeCommand() with a period
        static double computeCommand(control_toolbox::Pid & pid, double error, const rclcpp::Duration & period)
        {
            return pid.computeCommand(error, static_cast<uint64_t>(period.nanoseconds()));
        }
        static double computeCommand(control_toolbox::Pid & pid, double error, double error_dot, const rclcpp::Duration & period)
        {
            return pid.computeCommand(error, error_dot, static_cast<uint64_t>(period.nanoseconds()));
        }
	};

    inline CallbackReturn PIDKinematicChainControllerBase::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        if (KinematicChainControllerBase::on_configure(previous_state) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        PIDs_.clear();
        PIDs_.resize(kdl_chain_.getNrOfJoints());

        // Parsing PID gains from YAML, e.g. pid_lwr_a1_joint: {p: 250, i: 10, d: 30, i_clamp_min: -0.3, i_clamp_max: 0.3}
        std::string pid_ = ("pid_");
        for (size_t i = 0; i < chain_joint_names_.size(); ++i)
        {
            const std::string prefix = pid_ + chain_joint_names_[i] + ".";
            const double p = getParamDouble(prefix + "p", 0.0);
            const double i_gain = getParamDouble(prefix + "i", 0.0);
            const double d = getParamDouble(prefix + "d", 0.0);
            const double i_clamp = getParamDouble(prefix + "i_clamp", 0.0);
            const double i_clamp_max = getParamDouble(prefix + "i_clamp_max", i_clamp);
            const double i_clamp_min = getParamDouble(prefix + "i_clamp_min", -i_clamp);
            PIDs_[i].initPid(p, i_gain, d, i_clamp_max, i_clamp_min);
        }

        return CallbackReturn::SUCCESS;
    }

}

#endif
