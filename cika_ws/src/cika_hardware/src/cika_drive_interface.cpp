#include "cika_hardware/cika_drive_interface.hpp"
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


namespace cika_hardware
{
    bool CikaDriveInterface::try_reconnect()
{
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    const char* esp32_ip  = "10.245.199.91";
    const int   esp32_port = 8888;

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) return false;

    int flag = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(esp32_port);
    inet_pton(AF_INET, esp32_ip, &addr.sin_addr);

    // blocking connect with short timeout
    // struct timeval tv{ .tv_sec = 2, .tv_usec = 0 };
    struct timeval tv;
    tv.tv_sec  = 2;
    tv.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // restore non-blocking for reads
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    serial_buffer_.clear();
    RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"), "Reconnected to ESP32.");
    return true;
}
    // ── on_init ──────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaDriveInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        if (info_.joints.size() < 4) {
            RCLCPP_ERROR(rclcpp::get_logger("CikaDriveInterface"),
                         "Expected at least 4 joints, got %zu.", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const std::array<std::string, 4> expected_joints = {
            "left_front_wheel_1_joint", "left_back_wheel_1_joint",
            "right_front_wheel_1_joint", "right_back_wheel_1_joint"};
        for (std::size_t i = 0; i < 4; ++i) {
            if (info_.joints[i].name != expected_joints[i]) {
                RCLCPP_ERROR(rclcpp::get_logger("CikaDriveInterface"),
                             "Joint[%zu] expected '%s', got '%s'.",
                             i, expected_joints[i].c_str(), info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        node_     = rclcpp::Node::make_shared("cika_drive_interface_node");
        imu_pub_  = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/raw", 10);
        mag_pub_  = node_->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
        tf_static_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = node_->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id  = "imu_link";
        t.transform.translation.z = 0.05;
        t.transform.rotation.w    = 1.0;
        tf_static_->sendTransform(t);

        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"),
                    "Initialized Cika Drive Interface (WiFi TCP Mode)");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── export_state_interfaces ───────────────────────────────────────────────────
    std::vector<hardware_interface::StateInterface>
    CikaDriveInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;
        for (std::size_t i = 0; i < 4; ++i) {
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i * 2]);
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,  &hw_states_[i * 2 + 1]);
        }
        for (std::size_t i = 4; i < info_.joints.size(); ++i) {
            const std::size_t idx = i - 4;
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_arm_states_[idx * 2]);
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,  &hw_arm_states_[idx * 2 + 1]);
        }
        return interfaces;
    }

    // ── export_command_interfaces ─────────────────────────────────────────────────
    std::vector<hardware_interface::CommandInterface>
    CikaDriveInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (std::size_t i = 0; i < 4; ++i)
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
        for (std::size_t i = 4; i < info_.joints.size(); ++i)
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_arm_commands_[i - 4]);
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

        // ── Open TCP socket to ESP32 ──────────────────────────────────────────
        const char* esp32_ip = "10.245.199.91"; // UPDATE after flashing ESP32
        const int   esp32_port = 8888;

        last_reconnect_attempt_ = node_->get_clock()->now() - rclcpp::Duration(2, 0);

        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("CikaDriveInterface"), "Failed to create socket");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Disable Nagle — we want commands sent immediately
        int flag = 1;
        setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        // Non-blocking read so the control loop never hangs
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(esp32_port);
        inet_pton(AF_INET, esp32_ip, &addr.sin_addr);

        // Temporarily set blocking for connect
        fcntl(socket_fd_, F_SETFL, flags);
        if (connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("CikaDriveInterface"),
                         "Failed to connect to ESP32 at %s:%d", esp32_ip, esp32_port);
            close(socket_fd_);
            socket_fd_ = -1;
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Restore non-blocking
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"),
                    "Connected to ESP32 TCP server at %s:%d", esp32_ip, esp32_port);
        last_enc_time_ = node_->get_clock()->now();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── on_deactivate ─────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaDriveInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        if (socket_fd_ != -1) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"), "Deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── read ──────────────────────────────────────────────────────────────────────
    hardware_interface::return_type CikaDriveInterface::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (socket_fd_ == -1) {
            auto now = node_->get_clock()->now();
            if ((now - last_reconnect_attempt_).seconds() > 1.0) {
            last_reconnect_attempt_ = now;
            if (!try_reconnect()) {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CikaDriveInterface"),
                    *node_->get_clock(), 5000, "ESP32 unreachable, retrying...");
                }
            }
            return hardware_interface::return_type::OK;
        }

        char buf[512];
        ssize_t n = recv(socket_fd_, buf, sizeof(buf) - 1, 0);

        if (n == 0) {
            // peer closed connection cleanly
            RCLCPP_WARN(rclcpp::get_logger("CikaDriveInterface"), "ESP32 disconnected.");
            close(socket_fd_);
            socket_fd_ = -1;
            return hardware_interface::return_type::OK;
        }

        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            // real socket error
            RCLCPP_WARN(rclcpp::get_logger("CikaDriveInterface"),
                        "recv error %d, reconnecting.", errno);
            close(socket_fd_);
            socket_fd_ = -1;
            return hardware_interface::return_type::OK;
        }

        if (n > 0) {
            buf[n] = '\0';
            serial_buffer_ += buf;

            size_t pos;
            while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
                std::string line = serial_buffer_.substr(0, pos);
                serial_buffer_.erase(0, pos + 1);

                if (line.rfind("E:", 0) == 0) {
                    double p_lf, p_lb, p_rf, p_rb;
                    if (sscanf(line.c_str(), "E:%lf,%lf,%lf,%lf",
                               &p_lf, &p_lb, &p_rf, &p_rb) == 4) {
                        auto current_time = node_->get_clock()->now();
                        double actual_dt  = (current_time - last_enc_time_).seconds();

                        double prev_lf = hw_states_[0], prev_lb = hw_states_[2];
                        double prev_rf = hw_states_[4], prev_rb = hw_states_[6];

                        hw_states_[0] = -p_lf; hw_states_[2] = -p_lb;
                        hw_states_[4] = -p_rf; hw_states_[6] = -p_rb;

                        if (actual_dt > 0.0 && actual_dt <= 0.2) {
                            hw_states_[1] = (hw_states_[0] - prev_lf) / actual_dt;
                            hw_states_[3] = (hw_states_[2] - prev_lb) / actual_dt;
                            hw_states_[5] = (hw_states_[4] - prev_rf) / actual_dt;
                            hw_states_[7] = (hw_states_[6] - prev_rb) / actual_dt;
                        } else {
                            hw_states_[1] = hw_states_[3] = hw_states_[5] = hw_states_[7] = 0.0;
                        }
                        last_enc_time_ = current_time;
                    }
                } else if (line.rfind("I:", 0) == 0) {
                    double ax, ay, az, gx, gy, gz, mx, my, mz;
                    if (sscanf(line.c_str(),
                               "I:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                               &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz) == 9) {
                        auto stamp = node_->get_clock()->now();

                        sensor_msgs::msg::Imu imu_msg;
                        imu_msg.header.stamp    = stamp;
                        imu_msg.header.frame_id = "imu_link";
                        imu_msg.linear_acceleration.x = ax;
                        imu_msg.linear_acceleration.y = ay;
                        imu_msg.linear_acceleration.z = az;
                        imu_msg.angular_velocity.x =  gx;
                        imu_msg.angular_velocity.y =  gy;
                        imu_msg.angular_velocity.z = -gz;
                        imu_msg.orientation_covariance[0]       = -1.0;
                        imu_msg.linear_acceleration_covariance[0] = 0.01;
                        imu_msg.linear_acceleration_covariance[4] = 0.01;
                        imu_msg.linear_acceleration_covariance[8] = 0.01;
                        imu_msg.angular_velocity_covariance[0]  = 0.005;
                        imu_msg.angular_velocity_covariance[4]  = 0.005;
                        imu_msg.angular_velocity_covariance[8]  = 0.005;
                        imu_pub_->publish(imu_msg);

                        if (mx != 0.0 || my != 0.0 || mz != 0.0) {
                            sensor_msgs::msg::MagneticField mag_msg;
                            mag_msg.header.stamp    = stamp;
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
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (socket_fd_ != -1) {
            char buffer[64];
            int len = snprintf(buffer, sizeof(buffer), "<%.3f,%.3f,%.3f,%.3f>\n",
                               hw_commands_[0], hw_commands_[1],
                               hw_commands_[2], hw_commands_[3]);
            send(socket_fd_, buffer, len, MSG_NOSIGNAL);
        }
        return hardware_interface::return_type::OK;
    }

} // namespace cika_hardware

PLUGINLIB_EXPORT_CLASS(
    cika_hardware::CikaDriveInterface,
    hardware_interface::SystemInterface)