#include "cika_hardware/cika_drive_interface.hpp"

#include <cmath>
#include <cstring>
#include <fcntl.h>   // POSIX File control definitions
#include <termios.h> // POSIX Terminal control definitions
#include <unistd.h>  // UNIX Standard function definitions
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

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

        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"), "Initialized Cika Drive Interface (Raw Serial Mode)");

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

        // Open the raw serial port to the ESP32
        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CikaDriveInterface"), "Failed to open /dev/ttyUSB0");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Configure serial port for 115200 baud, 8N1
        struct termios options;
        tcgetattr(serial_fd_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
        options.c_cflag &= ~PARENB;          // No parity
        options.c_cflag &= ~CSTOPB;          // 1 stop bit
        options.c_cflag &= ~CSIZE;           // Mask character size bits
        options.c_cflag |= CS8;              // 8 data bits

        // Raw mode (no canonical processing, no echo, no signals)
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
        options.c_oflag &= ~OPOST;                  // Raw output

        tcsetattr(serial_fd_, TCSANOW, &options);

        RCLCPP_INFO(rclcpp::get_logger("CikaDriveInterface"), "Activated Raw UART Bridge on /dev/ttyUSB0.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── on_deactivate ─────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaDriveInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        // Close the serial port safely
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
        // Read available bytes non-blocking
        int n = ::read(serial_fd_, buf, sizeof(buf) - 1);

        if (n > 0)
        {
            buf[n] = '\0';
            serial_buffer_ += buf; // Append to our persistent string buffer

            // Process all complete lines found in the buffer
            size_t pos;
            while ((pos = serial_buffer_.find('\n')) != std::string::npos)
            {
                std::string line = serial_buffer_.substr(0, pos);
                serial_buffer_.erase(0, pos + 1); // Remove processed line

                // Check if it's an Encoder (E:) message from the ESP32
                if (line.rfind("E:", 0) == 0)
                {
                    double p_lf, p_lb, p_rf, p_rb;

                    if (sscanf(line.c_str(), "E:%lf,%lf,%lf,%lf", &p_lf, &p_lb, &p_rf, &p_rb) == 4)
                    {
                        // Calculate velocity = (new_position - old_position) / dt
                        double dt = period.seconds();
                        if (dt > 0.0)
                        {
                            hw_states_[1] = (p_lf - hw_states_[0]) / dt; // vel_lf
                            hw_states_[3] = (p_lb - hw_states_[2]) / dt; // vel_lb
                            hw_states_[5] = (p_rf - hw_states_[4]) / dt; // vel_rf
                            hw_states_[7] = (p_rb - hw_states_[6]) / dt; // vel_rb
                        }

                        // Update current position states
                        hw_states_[0] = p_lf; // pos_lf
                        hw_states_[2] = p_lb; // pos_lb
                        hw_states_[4] = p_rf; // pos_rf
                        hw_states_[6] = p_rb; // pos_rb
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
            // Format: <vel_lf, vel_lb, vel_rf, vel_rb>\n
            snprintf(buffer, sizeof(buffer), "<%.3f,%.3f,%.3f,%.3f>\n",
                     hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3]);

            // Send string to the ESP32
            ::write(serial_fd_, buffer, strlen(buffer));
        }

        return hardware_interface::return_type::OK;
    }

} // namespace cika_hardware

PLUGINLIB_EXPORT_CLASS(
    cika_hardware::CikaDriveInterface,
    hardware_interface::SystemInterface)