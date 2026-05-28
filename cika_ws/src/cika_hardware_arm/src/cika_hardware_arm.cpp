// Serial Communication Includes
#include <fcntl.h>    // File controls like O_RDWR
#include <errno.h>    // Error integer and strerror()
#include <unistd.h>   // write(), read(), close()
#include <chrono>
#include <thread>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "cika_hardware_arm/cika_hardware_arm.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cika_hardware_arm
{
    // ─────────────────────────────────────────────
    // on_init
    // ─────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaHardware::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_states_position_.assign(info_.joints.size(), 0.0);
        hw_states_velocity_.assign(info_.joints.size(), 0.0);
        hw_commands_.assign(info_.joints.size(), 0.0);

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("CikaHardware"), "Joint '%s' missing position command interface.", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2 || 
                joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
                joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(rclcpp::get_logger("CikaHardware"), "Joint '%s' missing position/velocity state interfaces.", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ─────────────────────────────────────────────
    // on_configure
    // ─────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (uint i = 0; i < hw_states_position_.size(); i++)
        {
            hw_states_position_[i] = 0.0;
            hw_states_velocity_[i] = 0.0;
            hw_commands_[i] = 0.0;
        }

        RCLCPP_INFO(rclcpp::get_logger("CikaHardware"), "Successfully configured!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ─────────────────────────────────────────────
    // on_activate
    // ─────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        std::string port_name = "/dev/ttyARMESP32";

        SerialPort = open(port_name.c_str(), O_RDWR);
        if (SerialPort < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CikaHardware"), "Error opening serial port %s: %s", port_name.c_str(), strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (tcgetattr(SerialPort, &tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CikaHardware"), "Error getting terminal attributes: %s", strerror(errno));
            close(SerialPort);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Control Modes
        tty.c_cflag &= ~PARENB;         // No parity
        tty.c_cflag &= ~CSTOPB;         // One stop bit
        tty.c_cflag &= ~CSIZE;          // Clear size bits
        tty.c_cflag |= CS8;             // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;  // Enable read, ignore modem lines

        // Local Modes
        tty.c_lflag &= ~ICANON;         // Raw mode
        tty.c_lflag &= ~ECHO;           // No echo
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;           // No signal chars

        // Input Modes
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Raw input

        // Output Modes
        tty.c_oflag &= ~OPOST;          // Raw output
        tty.c_oflag &= ~ONLCR;          // No newline conversion

        // Timing
        tty.c_cc[VTIME] = 1;            // 0.1s timeout
        tty.c_cc[VMIN] = 0;             // Non-blocking read

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tcflush(SerialPort, TCIFLUSH);

        if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CikaHardware"), "Error setting terminal attributes: %s", strerror(errno));
            close(SerialPort);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("CikaHardware"), "Serial port %s opened. Waiting 3s for ESP32 to boot...", port_name.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        RCLCPP_INFO(rclcpp::get_logger("CikaHardware"), "Successfully activated!");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ─────────────────────────────────────────────
    // on_deactivate
    // ─────────────────────────────────────────────
    hardware_interface::CallbackReturn CikaHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (SerialPort >= 0)
        {
            close(SerialPort);
            SerialPort = -1;
            RCLCPP_INFO(rclcpp::get_logger("CikaHardware"), "Serial port closed.");
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ─────────────────────────────────────────────
    // export_interfaces
    // ─────────────────────────────────────────────
    std::vector<hardware_interface::StateInterface> CikaHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> CikaHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    // ─────────────────────────────────────────────
    // Serial Helpers
    // ─────────────────────────────────────────────
    int CikaHardware::writeToSerial(unsigned char* buf, int nBytes)
    {
        return ::write(SerialPort, buf, nBytes);
    }

    int CikaHardware::ReadSerial(unsigned char* buf, int nBytes)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        int n = 0;
        while (n < nBytes)
        {
            int ret = ::read(SerialPort, &(buf[n]), 1);
            if (ret < 0) return ret;

            n += ret;

            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

            if (elapsed_ms > 10000) break; // 10 second timeout
        }
        return n;
    }

    // ─────────────────────────────────────────────
    // read
    // ─────────────────────────────────────────────
    hardware_interface::return_type CikaHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (uint i = 0; i < hw_states_position_.size(); i++)
        {
            hw_states_position_[i] = hw_commands_[i];
            hw_states_velocity_[i] = 0.0;
        }

        
        return hardware_interface::return_type::OK;
    }

    // ─────────────────────────────────────────────
    // write
    // ─────────────────────────────────────────────
    hardware_interface::return_type CikaHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (SerialPort < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CikaHardware"), "Serial port not open!");
            return hardware_interface::return_type::ERROR;
        }

        std::string cmd = "";
        for (uint i = 0; i < hw_commands_.size(); i++)
        {
            // 1. Convert radians to degrees
            double degrees_float = (hw_commands_[i] * 180.0 / M_PI);

            

            // 2. Exception handling for the upper arm servo (-180 to 0 rad range)
            // upside down mount, flip direction
            if (i == 3) degrees_float = -degrees_float;

            // 3. Round to nearest integer
            int degrees = static_cast<int>(std::round(degrees_float));

            // 4. Clamp to safe servo range (0 to 180)
            if (i != 0) 
            {
                degrees = std::max(0, std::min(180, degrees));
            }

            // 5. Append to command string
            cmd += "S" + std::to_string(i) + ":" + std::to_string(degrees);
            if (i < hw_commands_.size() - 1) cmd += ",";
        }
        cmd += "\n";

        // 6. Send over serial
        unsigned char buf[256];
        memcpy(buf, cmd.c_str(), cmd.size());
        int written = writeToSerial(buf, cmd.size());

        if (written < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CikaHardware"), "Failed to write to serial: %s", strerror(errno));
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

} // namespace cika_hardware_arm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cika_hardware_arm::CikaHardware, hardware_interface::SystemInterface)