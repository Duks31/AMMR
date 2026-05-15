// Serial Communication Includes from image_757216.png
#include <fcntl.h>   // File controls like O_RDWR
#include <errno.h>   // Error integer and strerror()
#include <unistd.h>  // write(), read(), close()
#include <chrono>

#include "cika_hardware_arm/cika_hardware_arm.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cika_hardware_arm
{
    hardware_interface::CallbackReturn CikaHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize vectors with 0.0 to prevent sending garbage data to the ESP32
    hw_states_position_.assign(info_.joints.size(), 0.0);
    hw_states_velocity_.assign(info_.joints.size(), 0.0);
    hw_commands_.assign(info_.joints.size(), 0.0);

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
            if (joint.command_interfaces.size() != 1)
            {
            RCLCPP_FATAL(rclcpp::get_logger("HardwareInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
            RCLCPP_FATAL(rclcpp::get_logger("CikaHardware"),
                "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
            RCLCPP_FATAL(rclcpp::get_logger("CikaHardware"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
            RCLCPP_FATAL(rclcpp::get_logger("CikaHardware"),
                "Joint '%s' has '%s' state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
            RCLCPP_FATAL(rclcpp::get_logger("CikaHardware"),
                "Joint '%s' has '%s' state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
            }

        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn CikaHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
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
    hardware_interface::CallbackReturn CikaHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        std::string port_name = "/dev/ttyUSB0"; // Adjust as necessary
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
        // --- FLAGS FROM IMAGE_6B1B72.JPG ---

        // Control Modes (c_cflag)
        tty.c_cflag &= ~PARENB;         // Clear parity bit
        tty.c_cflag &= ~CSTOPB;         // Use only one stop bit
        tty.c_cflag &= ~CSIZE;          // Clear size bits
        tty.c_cflag |= CS8;             // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines

        // Local Modes (c_lflag)
        tty.c_lflag &= ~ICANON;         // Disable canonical mode (handle data as-is)
        tty.c_lflag &= ~ECHO;           // Disable echo
        tty.c_lflag &= ~ECHOE;          // Disable erasure echo
        tty.c_lflag &= ~ECHONL;         // Disable new-line echo
        tty.c_lflag &= ~ISIG;           // Disable interpretation of INTR, QUIT, SUSP

        // Input Modes (c_iflag)
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes

        // Output Modes (c_oflag)
        tty.c_oflag &= ~OPOST;          // Prevent special handling of output bytes
        tty.c_oflag &= ~ONLCR;          // Prevent conversion of newline to carriage return/line feed

        // VTIME and VMIN (Timing settings)
        tty.c_cc[VTIME] = 1;            // Wait for up to 0.1s (1 decisecond)
        tty.c_cc[VMIN] = 0;

        speed_t baud_rate = B115200; // Set baud rate to 115200
        cfsetispeed(&tty, baud_rate);
        cfsetospeed(&tty, baud_rate);

        tcflush(SerialPort, TCIFLUSH); // Flush any data received but not read

        if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CikaHardware"), "Error setting terminal attributes: %s", strerror(errno));
            close(SerialPort);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("CikaHardware"), "Successfully activated and opened serial port %s", port_name.c_str());


        // Start the timer
        auto t_start = std::chrono::high_resolution_clock::now();

        while (true)
        {
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            
            // Logic from image_6aaa3a.png: Wait for 3000ms (3 seconds)
            if (elapsed_time_ms > 3000)
            {
                break;
            }
                
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CikaHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;

    }

    std::vector<hardware_interface::StateInterface> CikaHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> CikaHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    int CikaHardware::writeToSerial(unsigned char* buf, int nBytes)
    {
        return  ::write(SerialPort, buf, nBytes);
    }

    int CikaHardware::ReadSerial(unsigned char* buf, int nBytes)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        int n = 0;
        while (n < nBytes)
        {
            // Read 1 byte at a time into the buffer
            int ret = ::read(SerialPort, &(buf[n]), 1);
            if (ret < 0)
            {
                return ret; // Return the error code
            }
        
            n += ret;
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            // Timeout check: 10,000ms = 10 seconds

            if (elapsed_time_ms > 10000)
            {
                break;
            }
        }
        return n; 
    }

    hardware_interface::return_type CikaHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CikaHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cika_hardware_arm::CikaHardware, hardware_interface::SystemInterface)