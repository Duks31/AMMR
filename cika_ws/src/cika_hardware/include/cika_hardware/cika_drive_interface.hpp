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

        // Lifecycle
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

        // Control loop — called at update_rate (100 Hz per controllers.yaml)
        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

    private:
        // ── Raw Serial Bridge ─────────────────────────────────────────────────────────
        int serial_fd_{-1};
        std::string serial_buffer_;

        // ── ROS2 node handle + publishers ─────────────────────────────────────────────
        // get_node_base_interface() gives us access to the lifecycle node's clock
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

        // ── Drive joint state/command storage ─────────────────────────────────────────
        // hw_states_ layout (8 doubles):
        //   [pos_lf, vel_lf, pos_lb, vel_lb, pos_rf, vel_rf, pos_rb, vel_rb]
        std::array<double, 8> hw_states_{};

        // hw_commands_ layout (4 doubles):
        //   [vel_lf, vel_lb, vel_rf, vel_rb]
        std::array<double, 4> hw_commands_{};

        // ── Dummy storage for arm + gripper joints ────────────────────────────────────
        // (7 joints × 2 = 14 state values, 7 command values).
        // These are exported as zero-valued interfaces so the controller_manager
        // does not crash at startup. They will be replaced by CikaArmInterface.
        //
        // Joint order matches the URDF ros2_control block (indices 4–10):
        //   4: turntable_1_joint
        //   5: lower_arm_1_joint
        //   6: center_arm_1_joint
        //   7: upper_arm_1_joint
        //   8: gripper_mount_1_joint
        //   9: left_gripper_1_joint
        //  10: right_gripper_1_joint
        std::array<double, 14> hw_arm_states_{};  // [pos_j, vel_j] × 7
        std::array<double, 7> hw_arm_commands_{}; // [pos_j] × 7

        // ── Robot geometry ────────────────────────────────────────────────────────────
        double wheel_separation_{0.363};
        double wheel_radius_{0.0885};

        rclcpp::Time last_enc_time_;
    };

} // namespace cika_hardware

#endif // CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_


// #ifndef CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_
// #define CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_

// #include <array>
// #include <memory>
// #include <string>
// #include <vector>

// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/handle.hpp"
// #include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/state.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/magnetic_field.hpp"
// #include "tf2_ros/static_transform_broadcaster.h"

// namespace cika_hardware
// {

// class CikaDriveInterface : public hardware_interface::SystemInterface
// {
// public:
//     CikaDriveInterface() = default;

//     hardware_interface::CallbackReturn on_init(
//         const hardware_interface::HardwareInfo &info) override;

//     std::vector<hardware_interface::StateInterface>
//     export_state_interfaces() override;

//     std::vector<hardware_interface::CommandInterface>
//     export_command_interfaces() override;

//     hardware_interface::CallbackReturn on_activate(
//         const rclcpp_lifecycle::State &previous_state) override;

//     hardware_interface::CallbackReturn on_deactivate(
//         const rclcpp_lifecycle::State &previous_state) override;

//     hardware_interface::return_type read(
//         const rclcpp::Time &time,
//         const rclcpp::Duration &period) override;

//     hardware_interface::return_type write(
//         const rclcpp::Time &time,
//         const rclcpp::Duration &period) override;

// private:
//     // ── TCP Socket ───────────────────────────────────────────────────────────
//     int socket_fd_{-1};
//     std::string serial_buffer_;

//     // ── ROS2 node + publishers ───────────────────────────────────────────────
//     rclcpp::Node::SharedPtr node_;
//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
//     rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
//     std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

//     // ── Drive joint state/command storage ────────────────────────────────────
//     // [pos_lf, vel_lf, pos_lb, vel_lb, pos_rf, vel_rf, pos_rb, vel_rb]
//     std::array<double, 8> hw_states_{};
//     // [vel_lf, vel_lb, vel_rf, vel_rb]
//     std::array<double, 4> hw_commands_{};

//     // ── Arm joint storage (dummy — handled by CikaArmInterface) ─────────────
//     std::array<double, 14> hw_arm_states_{};
//     std::array<double, 7>  hw_arm_commands_{};

//     // ── Geometry ─────────────────────────────────────────────────────────────
//     double wheel_separation_{0.363};
//     double wheel_radius_{0.0885};

//     rclcpp::Time last_enc_time_;

//     bool try_reconnect();
//     rclcpp::Time last_reconnect_attempt_;
// };

// } // namespace cika_hardware

// #endif // CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_