#include "so_arm_100_hardware/so_arm_100_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace so_arm_100_controller
{
    SOARM100Interface::SOARM100Interface()
    {
    }

    SOARM100Interface::~SOARM100Interface()
    {
        st3215_.end();
    }

    CallbackReturn SOARM100Interface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        for (const auto &joint : info_.joints)
        {
            int motor_id = 0;
            int zero_position = 2048;
            int servo_direction = 1;
            ControlMode control_mode = ControlMode::POSITION;

            for (const auto &param : joint.parameters)
            {
                if (param.first == "motor_id")
                    motor_id =
                        static_cast<int>(std::stoi(param.second));
                else if (param.first == "zero_position")
                    zero_position =
                        static_cast<int>(std::stoi(param.second));
                else if (param.first == "direction")
                {
                    servo_direction = static_cast<int>(std::stoi(param.second));
                    if (servo_direction != -1 && servo_direction != 1)
                    {
                        throw std::runtime_error("Invalid motor direction for joint: " +
                                                 joint.name);
                    }
                }
            }
            joints_[joint.name] = Joint{motor_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, zero_position, servo_direction, control_mode};
        }
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Initialized SOARM100Interface with %zu joints", info_.joints.size());
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SOARM100Interface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto &[name, joint] : joints_)
        {
            state_interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &joint.position);
            state_interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
            state_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &joint.effort);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SOARM100Interface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto &[name, joint] : joints_)
        {
            command_interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &joint.position_command);
            command_interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_command);
            command_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &joint.effort_command);
        }
        return command_interfaces;
    }

    CallbackReturn SOARM100Interface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Activating so_arm_100 hardware interface...");

        int serial_baudrate = info_.hardware_parameters.count("serial_baudrate") ? std::stoi(info_.hardware_parameters.at("serial_baudrate")) : 1000000;
        std::string serial_port = info_.hardware_parameters.count("serial_port") ? info_.hardware_parameters.at("serial_port") : "/dev/ttyUSB0";

        if (!st3215_.begin(serial_baudrate, serial_port.c_str()))
        {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), "Failed to initialize motors");
            return CallbackReturn::ERROR;
        }

        // Initialize each servo
        for (auto &[name, joint] : joints_)
        {
            uint8_t servo_id = joint.motor_id;

            // First ping the servo
            if (st3215_.Ping(servo_id) == -1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"),
                             "No response from servo %d during initialization", servo_id);
                return CallbackReturn::ERROR;
            }

            // Set to position control mode
            if (!st3215_.Mode(servo_id, 0))
            {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"),
                             "Failed to set mode for servo %d", servo_id);
                return CallbackReturn::ERROR;
            }

            // Read initial position and set command to match
            if (st3215_.FeedBack(servo_id) != -1)
            {
                int raw_pos = st3215_.ReadPos(servo_id);
                joint.position = joint.servo_direction * (raw_pos - joint.zero_position) * 2 * M_PI / 4096.0;
                joint.position_command = joint.position;

                joint.velocity = st3215_.ReadSpeed(servo_id) * 2 * M_PI / 4096.0;

                RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
                            "Servo %d initialized at position %d", servo_id, raw_pos);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SOARM100Interface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        for (auto &[name, joint] : joints_)
        {
            st3215_.EnableTorque(joint.motor_id, 0);
        }

        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SOARM100Interface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        for (auto &[name, joint] : joints_)
        {
            uint8_t servo_id = joint.motor_id;
            // Convert from radians (-π to π) to servo ticks (0-4095)
            int joint_pos_cmd = joint.zero_position + joint.servo_direction * (int)(joint.position_command * 4096.0 / (2 * M_PI));

            RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                         "Servo %d command: %.2f rad -> %d ticks",
                         servo_id, joint.position_command, joint_pos_cmd);

            if (!st3215_.RegWritePosEx(servo_id, joint_pos_cmd, 4500, 255))
            {
                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                            "Failed to write position %d to servo %d", joint_pos_cmd, servo_id);
            }
        }
        st3215_.RegWriteAction();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SOARM100Interface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        for (auto &[name, joint] : joints_)
        {
            uint8_t servo_id = joint.motor_id;

            // Add small delay between reads
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Increased delay

            // Full feedback read when torque is enabled
            if (st3215_.FeedBack(servo_id) != -1)
            {
                int raw_pos = st3215_.ReadPos(servo_id);
                joint.position = joint.servo_direction * (raw_pos - joint.zero_position) * 2 * M_PI / 4096.0;
                joint.velocity = st3215_.ReadSpeed(servo_id) * 2 * M_PI / 4096.0;

                double pwm = st3215_.ReadLoad(servo_id) / 10.0;
                int move = st3215_.ReadMove(servo_id);
                double temperature = st3215_.ReadTemper(servo_id);
                double voltage = st3215_.ReadVoltage(servo_id) / 10;
                double current = st3215_.ReadCurrent(servo_id) * 6.5 / 1000;

                RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                             "Servo %d: raw_pos=%d (%.2f rad) speed=%.2f pwm=%.2f temp=%.1f V=%.1f I=%.3f",
                             servo_id, raw_pos, joint.position, joint.velocity, pwm, temperature, voltage, current);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                            "Failed to read feedback from servo %d", servo_id);
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace so_arm_100_controller

PLUGINLIB_EXPORT_CLASS(so_arm_100_controller::SOARM100Interface, hardware_interface::SystemInterface)
