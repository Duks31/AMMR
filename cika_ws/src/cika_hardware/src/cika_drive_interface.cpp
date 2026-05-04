#include "cika_hardware/cika_drive_interface.hpp"

#include <cmath>
#include <thread>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cika_hardware
{

    // ── on_init ──────────────────────────────────────────────────────────────────
    // Validates the URDF ros2_control block and reads any <param> overrides.
    // The controller_manager calls this once when the plugin is loaded.
    hardware_interface::CallbackReturn CikaDriveInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // We only handle the 4 drive joints. The arm/gripper joints declared in the
        // same ros2_control block are managed by GazeboSimSystem in sim and will get
        // their own CikaArmInterface plugin in Phase 2. For now we validate that we
        // have at least 4 joints and that the first 4 are the wheel joints.
        if (info_.joints.size() < 4)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("CikaDriveInterface"),
                "Expected at least 4 joints, got %zu.", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate the 4 drive joints — each must have velocity command + pos/vel state
        const std::array<std::string, 4> expected_joints = {
            "left_front_wheel_1_joint",
            "left_back_wheel_1_joint",
            "right_front_wheel_1_joint",
            "right_back_wheel_1_joint"};

        for (std::size_t i = 0; i < 4; ++i)
        {
            if (info_.joints[i].name != expected_joints[i])
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("CikaDriveInterface"),
                    "Joint[%zu] expected '%s', got '%s'. Check URDF joint order.",
                    i, expected_joints[i].c_str(), info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (info_.joints[i].command_interfaces.size() != 1 ||
                info_.joints[i].command_interfaces[0].name !=
                    hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("CikaDriveInterface"),
                    "Joint '%s' must have exactly one velocity command interface.",
                    info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Read optional URDF <param> overrides
        auto it = info_.hardware_parameters.find("wheel_separation");
        if (it != info_.hardware_parameters.end())
        {
            wheel_separation_ = std::stod(it->second);
        }
        it = info_.hardware_parameters.find("wheel_radius");
        if (it != info_.hardware_parameters.end())
        {
            wheel_radius_ = std::stod(it->second);
        }
        it = info_.hardware_parameters.find("odom_topic");
        if (it != info_.hardware_parameters.end())
        {
            odom_topic_ = it->second;
        }
        it = info_.hardware_parameters.find("cmd_topic");
        if (it != info_.hardware_parameters.end())
        {
            cmd_topic_ = it->second;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("CikaDriveInterface"),
            "Initialized — wheel_separation=%.4f m  wheel_radius=%.4f m",
            wheel_separation_, wheel_radius_);
        RCLCPP_INFO(
            rclcpp::get_logger("CikaDriveInterface"),
            "odom_topic='%s'  cmd_topic='%s'",
            odom_topic_.c_str(), cmd_topic_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── export_state_interfaces ───────────────────────────────────────────────────
    // Exports position+velocity state for all 11 joints in the ros2_control block.
    // Joints 0-3: real drive wheel interfaces backed by odometry.
    // Joints 4-10: dummy arm/gripper interfaces (zero) until CikaArmInterface.
    std::vector<hardware_interface::StateInterface>
    CikaDriveInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;

        // Drive wheels (joints 0-3)
        for (std::size_t i = 0; i < 4; ++i)
        {
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_states_[i * 2]);
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_states_[i * 2 + 1]);
        }

        // Arm + gripper joints (joints 4-10) — dummy zeros
        for (std::size_t i = 4; i < info_.joints.size(); ++i)
        {
            const std::size_t idx = i - 4;
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_arm_states_[idx * 2]);
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_arm_states_[idx * 2 + 1]);
        }

        return interfaces;
    }

    // ── export_command_interfaces ─────────────────────────────────────────────────
    // Exports velocity commands for drive joints, position commands for arm/gripper.
    std::vector<hardware_interface::CommandInterface>
    CikaDriveInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;

        // Drive wheels — velocity commands (joints 0-3)
        for (std::size_t i = 0; i < 4; ++i)
        {
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_commands_[i]);
        }

        // Arm + gripper — position commands (joints 4-10) — dummy zeros
        for (std::size_t i = 4; i < info_.joints.size(); ++i)
        {
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_arm_commands_[i - 4]);
        }

        return interfaces;
    }

    // ── on_activate ───────────────────────────────────────────────────────────────
    // Creates the internal ROS2 node, publisher, subscriber, and a dedicated
    // background thread to spin the executor. This avoids calling spin_some()
    // inside read() which blocks the controller_manager at 1000 Hz.
    hardware_interface::CallbackReturn CikaDriveInterface::on_activate(
        const rclcpp_lifecycle::State &)
    {
        hw_states_.fill(0.0);
        hw_commands_.fill(0.0);
        hw_arm_states_.fill(0.0);
        hw_arm_commands_.fill(0.0);
        latest_linear_vel_ = 0.0;
        latest_angular_vel_ = 0.0;

        node_ = std::make_shared<rclcpp::Node>("cika_drive_hw_node");

        // Publisher: [vel_lf, vel_lb, vel_rf, vel_rb] rad/s → ESP32
        wheel_cmd_pub_ =
            node_->create_publisher<std_msgs::msg::Float32MultiArray>(
                cmd_topic_, rclcpp::QoS(10));

        // Subscriber: nav_msgs/Odometry ← ESP32
        odom_sub_ =
            node_->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_,
                rclcpp::QoS(10),
                std::bind(&CikaDriveInterface::odom_callback, this, std::placeholders::_1));

        // Spin the node on a dedicated background thread so read() and write()
        // are never blocked waiting for callbacks
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spin_thread_ = std::thread([this]()
                                   { executor_->spin(); });

        RCLCPP_INFO(
            rclcpp::get_logger("CikaDriveInterface"),
            "Activated. Listening on '%s', commanding on '%s'.",
            odom_topic_.c_str(), cmd_topic_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── on_deactivate ─────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaDriveInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        executor_.reset();
        node_.reset();
        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"), "Deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── read ──────────────────────────────────────────────────────────────────────
    // Called every control cycle (1000 Hz). Reads latest odom data written by
    // the background executor thread and converts to per-wheel velocities.
    // No spin_some here — that was the cause of the controller_manager blockage.
    hardware_interface::return_type CikaDriveInterface::read(
        const rclcpp::Time &,
        const rclcpp::Duration &period)
    {

        double vel_left = 0.0, vel_right = 0.0;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_to_wheel_velocities(
                latest_linear_vel_, latest_angular_vel_,
                vel_left, vel_right);
        }

        const double dt = period.seconds();

        // left_front
        hw_states_[0] += vel_left * dt; // position (integrated)
        hw_states_[1] = vel_left;       // velocity

        // left_back
        hw_states_[2] += vel_left * dt;
        hw_states_[3] = vel_left;

        // right_front
        hw_states_[4] += vel_right * dt;
        hw_states_[5] = vel_right;

        // right_back
        hw_states_[6] += vel_right * dt;
        hw_states_[7] = vel_right;

        return hardware_interface::return_type::OK;
    }

    // ── write ─────────────────────────────────────────────────────────────────────
    // Called every control cycle after read().
    // Publishes the velocity commands placed into hw_commands_ by skid_steer_controller.
    hardware_interface::return_type CikaDriveInterface::write(
        const rclcpp::Time &,
        const rclcpp::Duration &)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(4);
        msg.data[0] = static_cast<float>(hw_commands_[0]); // left_front
        msg.data[1] = static_cast<float>(hw_commands_[1]); // left_back
        msg.data[2] = static_cast<float>(hw_commands_[2]); // right_front
        msg.data[3] = static_cast<float>(hw_commands_[3]); // right_back
        wheel_cmd_pub_->publish(msg);

        return hardware_interface::return_type::OK;
    }

    // ── odom_callback (private) ───────────────────────────────────────────────────
    void CikaDriveInterface::odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_linear_vel_ = msg->twist.twist.linear.x;
        latest_angular_vel_ = msg->twist.twist.angular.z;
    }

    // ── odom_to_wheel_velocities (private) ───────────────────────────────────────
    // Inverse skid-steer kinematics: robot linear/angular → wheel rad/s
    void CikaDriveInterface::odom_to_wheel_velocities(
        double linear_x, double angular_z,
        double &vel_left, double &vel_right)
    {
        const double half_sep = wheel_separation_ / 2.0;
        vel_left = (linear_x - angular_z * half_sep) / wheel_radius_;
        vel_right = (linear_x + angular_z * half_sep) / wheel_radius_;
    }

} // namespace cika_hardware

PLUGINLIB_EXPORT_CLASS(
    cika_hardware::CikaDriveInterface,
    hardware_interface::SystemInterface)