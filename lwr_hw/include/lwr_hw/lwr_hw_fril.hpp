#ifndef LWR_HW__LWR_HW_FRIL_H
#define LWR_HW__LWR_HW_FRIL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// ROS
#include <hardware_interface/system_interface.hpp>

// FRIL remote hooks
#include <FastResearchInterface.h>

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK   2000
#define EOK 0

namespace lwr_hw
{

/**
 * Real LWR 4+ through the Stanford FRI library (FRIL), loaded by the controller manager with
 * <plugin>lwr_hw/LWRHWFRIL</plugin> in the <hardware> tag of the ros2_control URDF description.
 *
 * Hardware parameters: name, file (FRI driver init file), root_name, tip_name.
 */
class LWRHWFRIL : public hardware_interface::SystemInterface, public LWRHW
{

public:

  LWRHWFRIL() : LWRHW() {}
  ~LWRHWFRIL() {}

  void stop();
  void set_mode(){return;};

  void setInitFile(std::string init_file){init_file_ = init_file; file_set_ = true;};

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
  std::string init_file_;
  bool file_set_ = false;

  // low-level interface
  std::shared_ptr<FastResearchInterface> device_;
  int ResultValue = 0;
};

}

#endif
