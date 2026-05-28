#ifndef CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_
#define CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace cika_hardware
{

class CikaDriveInterface : public hardware_interface::SystemInterface
{
public:
    CikaDriveInterface() = default;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

private:
    // ── TCP Socket ───────────────────────────────────────────────────────────
    int socket_fd_{-1};
    std::string serial_buffer_;

    // ── ROS2 node + publishers ───────────────────────────────────────────────
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

    // ── Drive joint state/command storage ────────────────────────────────────
    // [pos_lf, vel_lf, pos_lb, vel_lb, pos_rf, vel_rf, pos_rb, vel_rb]
    std::array<double, 8> hw_states_{};
    // [vel_lf, vel_lb, vel_rf, vel_rb]
    std::array<double, 4> hw_commands_{};

    // ── Arm joint storage (dummy — handled by CikaArmInterface) ─────────────
    std::array<double, 14> hw_arm_states_{};
    std::array<double, 7>  hw_arm_commands_{};

    // ── Geometry ─────────────────────────────────────────────────────────────
    double wheel_separation_{0.363};
    double wheel_radius_{0.0885};

    rclcpp::Time last_enc_time_;
};

} // namespace cika_hardware

#endif // CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_