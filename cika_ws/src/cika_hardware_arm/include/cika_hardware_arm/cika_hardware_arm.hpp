#ifndef CIKA_HARDWARE_ARM__CIKA_HARDWARE_ARM_HPP_
#define CIKA_HARDWARE_ARM__CIKA_HARDWARE_ARM_HPP_

#include <termios.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "cika_hardware_arm/visibility_control.h"

namespace cika_hardware_arm
{
class CikaHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CikaHardware)

  CIKA_HARDWARE_ARM_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CIKA_HARDWARE_ARM_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CIKA_HARDWARE_ARM_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CIKA_HARDWARE_ARM_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CIKA_HARDWARE_ARM_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CIKA_HARDWARE_ARM_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CIKA_HARDWARE_ARM_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CIKA_HARDWARE_ARM_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_states_position_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_velocity_;
  int SerialPort = -1;
  struct termios tty;
  int writeToSerial(unsigned char* buf, int nBytes);
  int ReadSerial(unsigned char* buf, int nBytes);
}; // <--- Semicolon fixed here
} // namespace cika_hardware_arm

#endif // CIKA_HARDWARE_ARM__CIKA_HARDWARE_ARM_HPP_ // <--- #endif fixed here