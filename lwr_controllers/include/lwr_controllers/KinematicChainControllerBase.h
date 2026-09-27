#ifndef KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define KINEMATIC_CHAIN_CONTROLLER_BASE_H

#include <urdf/model.h>
#include <lwr_hw/lwr_hw.h>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/string.hpp>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace controller_interface
{
    /**
     * Thin wrapper around the loaned ros2_control interfaces of a joint, to keep the ROS 1 API
     * (getPosition(), getVelocity(), getEffort(), setCommand()) in the controllers.
     */
    struct JointHandle
    {
        std::string name;
        const hardware_interface::LoanedStateInterface * position = nullptr;
        const hardware_interface::LoanedStateInterface * velocity = nullptr;
        const hardware_interface::LoanedStateInterface * effort = nullptr;
        hardware_interface::LoanedCommandInterface * command = nullptr;

        const std::string & getName() const { return name; }
        double getPosition() const { return position ? position->get_value() : std::numeric_limits<double>::quiet_NaN(); }
        double getVelocity() const { return velocity ? velocity->get_value() : std::numeric_limits<double>::quiet_NaN(); }
        double getEffort() const { return effort ? effort->get_value() : std::numeric_limits<double>::quiet_NaN(); }
        void setCommand(double command_value) { if (command) command->set_value(command_value); }
    };

    /**
     * Hands the latest message received in a (non real-time) subscription callback over to the
     * real-time update() loop. take() returns each new message once, and nullptr otherwise.
     */
    template<class MsgT>
    class CommandInbox
    {
    public:
        void write(const std::shared_ptr<MsgT> & msg) { buffer_.writeFromNonRT(msg); }
        std::shared_ptr<MsgT> take()
        {
            std::shared_ptr<MsgT> msg = *buffer_.readFromRT();
            if (!msg || msg == last_)
                return nullptr;
            last_ = msg;
            return msg;
        }
        void reset() { buffer_.writeFromNonRT(nullptr); last_.reset(); }

    private:
        realtime_tools::RealtimeBuffer<std::shared_ptr<MsgT>> buffer_{nullptr};
        std::shared_ptr<MsgT> last_;
    };

    /**
     * Base class of the lwr controllers: it parses the URDF, builds the KDL chain between the
     * root_name and tip_name parameters, reads the joint limits and gets the joint handles.
     *
     * The URDF is taken from the robot_description parameter of the controller or, if empty, from
     * the (latched) topic given by the robot_description_topic parameter (default /robot_description).
     */
	class KinematicChainControllerBase: public ControllerInterface
	{
	public:
        // which command interface the controller claims on each joint of the chain
        enum class CommandType { NONE, POSITION, EFFORT };

		explicit KinematicChainControllerBase(CommandType command_type = CommandType::EFFORT, bool claim_impedance_interfaces = false)
            : command_type_(command_type), claim_impedance_interfaces_(claim_impedance_interfaces) {}
		~KinematicChainControllerBase() {}

        CallbackReturn on_init() override;
        InterfaceConfiguration command_interface_configuration() const override;
        InterfaceConfiguration state_interface_configuration() const override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

	protected:
		KDL::Chain kdl_chain_;
        KDL::Vector gravity_;
        KDL::JntArrayAcc joint_msr_states_, joint_des_states_;  // joint states (measured and desired)

		struct limits_
		{
			KDL::JntArray min;
			KDL::JntArray max;
			KDL::JntArray center;
		} joint_limits_;

        std::vector<std::string> chain_joint_names_;
		std::vector<JointHandle> joint_handles_;
        std::vector<JointHandle> joint_stiffness_handles_;
        std::vector<JointHandle> joint_damping_handles_;
        std::vector<JointHandle> joint_set_point_handles_;

        bool getHandles();

        // parameter helpers (parameters in the yaml files are declared automatically, and integer values are accepted as double)
        double getParamDouble(const std::string & name, double default_value);
        std::string getParamString(const std::string & name, const std::string & default_value);

        // publish from the real-time loop without blocking
        template<class MsgT>
        static void publishRT(const std::shared_ptr<realtime_tools::RealtimePublisher<MsgT>> & pub, const MsgT & msg)
        {
            if (pub && pub->trylock())
            {
                pub->msg_ = msg;
                pub->unlockAndPublish();
            }
        }

        // get the robot description from the parameter or from the topic
        std::string getRobotDescription();

        rclcpp::Logger logger() const { return get_node()->get_logger(); }

	private:
        CommandType command_type_;
        bool claim_impedance_interfaces_;
	};

}

#endif
