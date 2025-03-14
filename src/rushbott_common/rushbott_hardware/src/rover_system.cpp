// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rushbott_hardware/rover_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rushbott_hardware
{
hardware_interface::CallbackReturn RoverSystemHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.loop_rate = std::stod(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.msg_attempts = std::stoi(info_.hardware_parameters["msg_attempts"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

    cfg_.stepper_gear_ratio = std::stod(info_.hardware_parameters["stepper_gear_ratio"]);
    cfg_.stepper_enc_per_rev = std::stoi(info_.hardware_parameters["stepper_enc_per_rev"]);
    cfg_.stepper_step_per_rev = std::stoi(info_.hardware_parameters["stepper_step_per_rev"]);

    cfg_.bldc_enc_per_rev = std::stoi(info_.hardware_parameters["bldc_enc_per_rev"]);

    int stepper_count = 0;
    int bldc_count = 0;
    int servo_count = 0;

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {

        if (joint.name.find("arm") != std::string::npos &&
            joint.state_interfaces.size() == 2 &&
            joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY &&
            joint.command_interfaces.size() == 2 &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            joint.command_interfaces[1].name == hardware_interface::HW_IF_VELOCITY
        )
        {
            stepper_count++;
            cfg_.stepper_initial_values.emplace_back(std::stod(joint.parameters.at("initial_value")));
        }
        else if (joint.name.find("wheel") != std::string::npos &&
            joint.state_interfaces.size() == 1 &&
            joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY
        )
        {
            bldc_count++;
        }
        else if (joint.name.find("servo") != std::string::npos &&
            joint.state_interfaces.size() == 0 &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION
        )
        {
            servo_count++;
        }
        else
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' is incorrectly configured, check name or interfaces.",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    stepper_state_positions_.resize(stepper_count, std::numeric_limits<double>::quiet_NaN());
    stepper_state_velocities_.resize(stepper_count, std::numeric_limits<double>::quiet_NaN());
    stepper_cmd_positions_.resize(stepper_count, std::numeric_limits<double>::quiet_NaN());
    stepper_cmd_velocities_.resize(stepper_count, std::numeric_limits<double>::quiet_NaN());

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int stepper_index = 0;
    int bldc_index = 0;
    int servo_index = 0;

    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        if (info_.joints[i].name.find("arm") != std::string::npos)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &stepper_state_positions_[stepper_index]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &stepper_state_velocities_[stepper_index]));
            stepper_index++;
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int stepper_index = 0;
    int bldc_index = 0;
    int servo_index = 0;

    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        if (info_.joints[i].name.find("arm") != std::string::npos)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &stepper_cmd_positions_[stepper_index]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &stepper_cmd_velocities_[stepper_index]));
            stepper_index++;
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn RoverSystemHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring...");

    if (comms_.connected())
    {
        comms_.disconnect();
    }

    MotorPacket config_packet = {};

    double step_conversion = (cfg_.stepper_enc_per_rev*cfg_.stepper_gear_ratio)/(2*M_PI);

    for (auto i = 0u; i < stepper_state_positions_.size(); i++)
    {
        if (std::isnan(stepper_state_positions_[i]))
        {
            stepper_state_positions_[i] = cfg_.stepper_initial_values[i];
            stepper_state_velocities_[i] = 0;
            stepper_cmd_positions_[i] = cfg_.stepper_initial_values[i];
            stepper_cmd_velocities_[i] = 0;
        }

        config_packet.stepper_pos[i] = (stepper_state_positions_[i] * step_conversion);
    }

    if (!comms_.connect(config_packet, cfg_.device, cfg_.baud_rate, cfg_.msg_attempts, cfg_.timeout_ms))
    {
        return hardware_interface::CallbackReturn::FAILURE;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    if (comms_.connected())
    {
    comms_.disconnect();
    }
    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating...");

    if (!comms_.connected())
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Successfully activated!");

    for (auto i = 0u; i < stepper_state_positions_.size(); i++)
    {
        stepper_state_positions_[i] = stepper_cmd_positions_[i];
        stepper_state_velocities_[i] = stepper_cmd_velocities_[i];
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating...");
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverSystemHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
    if (!comms_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }

    double delta_seconds = period.seconds();

    MotorPacket encoder_packet = {};

    if (comms_.read_encoders(encoder_packet))
    {
        std::vector<double> last_positions = stepper_state_positions_;
        double step_conversion = (2*M_PI)/(cfg_.stepper_enc_per_rev*cfg_.stepper_gear_ratio);

        for (size_t i = 0; i < stepper_state_positions_.size(); i++)
        {
            stepper_state_positions_[i] = (encoder_packet.stepper_pos[i] * step_conversion);
            stepper_state_velocities_[i] = (stepper_state_positions_[i] - last_positions[i]) / delta_seconds;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type rushbott_hardware::RoverSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!comms_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }

    MotorPacket motor_packet = {};

    double step_conversion = (cfg_.stepper_enc_per_rev*cfg_.stepper_gear_ratio)/(2*M_PI);

    for (size_t i = 0; i < stepper_cmd_positions_.size(); i++)
    {
        motor_packet.stepper_pos[i] = static_cast<int16_t>(stepper_cmd_positions_[i] * step_conversion);
        motor_packet.stepper_vel[i] = static_cast<int16_t>(stepper_cmd_velocities_[i] * step_conversion);
    }

    comms_.set_motors(motor_packet);
    return hardware_interface::return_type::OK;
}
    

}  // namespace rushbott_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rushbott_hardware::RoverSystemHardware, hardware_interface::SystemInterface)