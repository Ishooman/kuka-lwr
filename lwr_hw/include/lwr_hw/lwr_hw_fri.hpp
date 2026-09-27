#ifndef LWR_HW__LWR_HW_REAL_H
#define LWR_HW__LWR_HW_REAL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// ROS
#include <hardware_interface/system_interface.hpp>
#include <std_msgs/msg/bool.hpp>

// FRI remote hooks
#include <atomic>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include <thread>
#include "fri/friudp.h"
#include "fri/friremote.h"

// ToDo: add timeouts to all sync-while's to KRL since the UDP connection might be lost and we will know

namespace lwr_hw
{

/**
 * Real LWR 4+ through the KUKA FRI (remote side), loaded by the controller manager with
 * <plugin>lwr_hw/LWRHWFRI</plugin> in the <hardware> tag of the ros2_control URDF description.
 *
 * Hardware parameters: name, port (49939), ip (192.168.0.10), root_name, tip_name,
 * estop_topic (/<name>/emergency_stop).
 */
class LWRHWFRI : public hardware_interface::SystemInterface, public LWRHW
{

public:

  LWRHWFRI() : LWRHW() {}
  ~LWRHWFRI();

  void setPort(int port){port_ = port; port_set_ = true;};
  void setIP(std::string hintToRemoteHost){hintToRemoteHost_ = hintToRemoteHost; ip_set_ = true;};
  float getSampleTime(){return sampling_rate_;};

  // ros2_control
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Init, read, and write, with FRI hooks
  bool init();
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void doSwitch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

private:

  // Parameters
  int port_;
  bool port_set_ = false;
  std::string hintToRemoteHost_;
  bool ip_set_ = false;

  // low-level interface
  std::shared_ptr<friRemote> device_;

  // FRI values
  FRI_QUALITY lastQuality_;
  FRI_CTRL lastCtrlScheme_;

  float sampling_rate_;

  std::unique_ptr<std::thread> KRCCommThread_;
  std::atomic<bool> stopKRCComm_{false};
  void KRCCommThreadCallback();

  void startFRI();
  void stopFRI();

  // Emergency stop: while it is pressed, the controller commands are ignored and the measured position is held
  std::string estop_topic_;
  std::atomic<bool> isStopPressed_{false};
  bool wasStopHandled_ = true;
  rclcpp::Node::SharedPtr estop_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr estop_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  std::unique_ptr<std::thread> estop_thread_;
  void holdMeasuredPosition();
};

}

#endif
