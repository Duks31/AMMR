#ifndef CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_
#define CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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

        // Control loop — called at update_rate (1000 Hz per controllers.yaml)
        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void odom_to_wheel_velocities(
            double linear_x, double angular_z,
            double &vel_left, double &vel_right);

        // ROS2 node owned by this interface
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_cmd_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // Background executor — spins the node without blocking the control loop
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
        std::thread spin_thread_;

        // Latest robot-level velocities written by odom_callback, read by read()
        std::mutex odom_mutex_;
        double latest_linear_vel_{0.0};
        double latest_angular_vel_{0.0};

        // Drive joint state/command storage
        // hw_states_ layout (8 doubles):
        //   [pos_lf, vel_lf, pos_lb, vel_lb, pos_rf, vel_rf, pos_rb, vel_rb]
        std::array<double, 8> hw_states_{};

        // hw_commands_ layout (4 doubles):
        //   [vel_lf, vel_lb, vel_rf, vel_rb]
        std::array<double, 4> hw_commands_{};

        // Dummy storage for arm + gripper joints (7 joints × 2 = 14 state values,
        // 7 command values). These are exported as zero-valued interfaces so the
        // controller_manager does not crash at startup. They will be replaced by
        // CikaArmInterface in Phase 2.
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

        // Robot geometry — defaults match controllers.yaml, overridable via URDF <param>
        double wheel_separation_{0.363};
        double wheel_radius_{0.0885};

        // Topic names — overridable via URDF <param>
        std::string odom_topic_{"/cika/odom"};
        std::string cmd_topic_{"/cika/drive/wheel_vel_cmd"};
    };

} // namespace cika_hardware

#endif // CIKA_HARDWARE__CIKA_DRIVE_INTERFACE_HPP_