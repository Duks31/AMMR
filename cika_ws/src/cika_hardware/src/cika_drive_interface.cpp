#include "cika_hardware/cika_drive_interface.hpp"
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace cika_hardware
{
    // ── on_init ──────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaDriveInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (info_.joints.size() < 4)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("CikaDriveInterface"),
                "Expected at least 4 joints, got %zu.", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
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
                    "Joint[%zu] expected '%s', got '%s'.",
                    i, expected_joints[i].c_str(), info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // ── Create a standalone node for publishing ───────────────────────────────
        // ros2_control hardware interfaces don't inherit from Node, so we spin up
        // a minimal node just for the IMU/mag publishers and static TF broadcaster.
        node_ = rclcpp::Node::make_shared("cika_drive_interface_node");
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/raw", 10);
        mag_pub_ = node_->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
        tf_static_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

        // Publish static transform: base_link → imu_link
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = node_->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "imu_link";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.05;
        t.transform.rotation.w = 1.0;
        tf_static_->sendTransform(t);

        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"),
                    "Initialized Cika Drive Interface (Raw Serial Mode) with IMU publisher");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── export_state_interfaces ───────────────────────────────────────────────────
    std::vector<hardware_interface::StateInterface>
    CikaDriveInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;
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
    std::vector<hardware_interface::CommandInterface>
    CikaDriveInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (std::size_t i = 0; i < 4; ++i)
        {
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_commands_[i]);
        }
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
    hardware_interface::CallbackReturn CikaDriveInterface::on_activate(
        const rclcpp_lifecycle::State &)
    {
        hw_states_.fill(0.0);
        hw_commands_.fill(0.0);
        hw_arm_states_.fill(0.0);
        hw_arm_commands_.fill(0.0);

        serial_fd_ = open("/dev/esp32", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CikaDriveInterface"),
                         "Failed to open /dev/esp32");
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        tcsetattr(serial_fd_, TCSANOW, &options);

        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"),
                    "Activated Raw UART Bridge on /dev/esp32.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── on_deactivate ─────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaDriveInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        if (serial_fd_ != -1)
        {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"), "Deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── read ──────────────────────────────────────────────────────────────────────
    hardware_interface::return_type CikaDriveInterface::read(
        const rclcpp::Time &,
        const rclcpp::Duration &period)
    {
        if (serial_fd_ == -1)
            return hardware_interface::return_type::ERROR;

        char buf[256];
        int n = ::read(serial_fd_, buf, sizeof(buf) - 1);
        if (n > 0)
        {
            buf[n] = '\0';
            serial_buffer_ += buf;

            size_t pos;
            while ((pos = serial_buffer_.find('\n')) != std::string::npos)
            {
                std::string line = serial_buffer_.substr(0, pos);
                serial_buffer_.erase(0, pos + 1);

                // ── Encoder line ──────────────────────────────────────────────
                if (line.rfind("E:", 0) == 0)
                {
                    double p_lf, p_lb, p_rf, p_rb;
                    if (sscanf(line.c_str(), "E:%lf,%lf,%lf,%lf",
                               &p_lf, &p_lb, &p_rf, &p_rb) == 4)
                    {
                        double dt = period.seconds();
                        if (dt > 0.0)
                        {
                            hw_states_[1] = (p_lf - hw_states_[0]) / dt;
                            hw_states_[3] = (p_lb - hw_states_[2]) / dt;
                            hw_states_[5] = (-p_rf - hw_states_[4]) / dt;
                            hw_states_[7] = (-p_rb - hw_states_[6]) / dt;
                        }
                        hw_states_[0] = p_lf;
                        hw_states_[2] = p_lb;
                        hw_states_[4] = -p_rf;
                        hw_states_[6] = -p_rb;
                    }
                }
                // ── IMU line ──────────────────────────────────────────────────
                else if (line.rfind("I:", 0) == 0)
                {
                    double ax, ay, az, gx, gy, gz, mx, my, mz;
                    if (sscanf(line.c_str(),
                               "I:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                               &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz) == 9)
                    {
                        auto stamp = node_->get_clock()->now();

                        // IMU message
                        sensor_msgs::msg::Imu imu_msg;
                        imu_msg.header.stamp = stamp;
                        imu_msg.header.frame_id = "imu_link";
                        imu_msg.linear_acceleration.x = ax;
                        imu_msg.linear_acceleration.y = ay;
                        imu_msg.linear_acceleration.z = az;
                        imu_msg.angular_velocity.x = gx;
                        imu_msg.angular_velocity.y = gy;
                        imu_msg.angular_velocity.z = gz;
                        imu_msg.orientation_covariance[0] = -1.0;
                        imu_msg.linear_acceleration_covariance[0] = 0.01;
                        imu_msg.linear_acceleration_covariance[4] = 0.01;
                        imu_msg.linear_acceleration_covariance[8] = 0.01;
                        imu_msg.angular_velocity_covariance[0] = 0.005;
                        imu_msg.angular_velocity_covariance[4] = 0.005;
                        imu_msg.angular_velocity_covariance[8] = 0.005;
                        imu_pub_->publish(imu_msg);

                        // Mag message — skip if zeroed (mag read failed on ESP32)
                        if (mx != 0.0 || my != 0.0 || mz != 0.0)
                        {
                            sensor_msgs::msg::MagneticField mag_msg;
                            mag_msg.header.stamp = stamp;
                            mag_msg.header.frame_id = "imu_link";
                            mag_msg.magnetic_field.x = mx * 1e-6;
                            mag_msg.magnetic_field.y = my * 1e-6;
                            mag_msg.magnetic_field.z = mz * 1e-6;
                            mag_msg.magnetic_field_covariance[0] = 1e-6;
                            mag_msg.magnetic_field_covariance[4] = 1e-6;
                            mag_msg.magnetic_field_covariance[8] = 1e-6;
                            mag_pub_->publish(mag_msg);
                        }
                    }
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

    // ── write ─────────────────────────────────────────────────────────────────────
    hardware_interface::return_type CikaDriveInterface::write(
        const rclcpp::Time &,
        const rclcpp::Duration &)
    {
        if (serial_fd_ != -1)
        {
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "<%.3f,%.3f,%.3f,%.3f>\n",
                     hw_commands_[0], hw_commands_[1],
                     hw_commands_[2], hw_commands_[3]);
            ::write(serial_fd_, buffer, strlen(buffer));
        }
        return hardware_interface::return_type::OK;
    }

} // namespace cika_hardware

PLUGINLIB_EXPORT_CLASS(
    cika_hardware::CikaDriveInterface,
    hardware_interface::SystemInterface)