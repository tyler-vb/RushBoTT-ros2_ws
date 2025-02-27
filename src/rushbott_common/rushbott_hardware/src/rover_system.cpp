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

    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.msg_attempts = std::stof(info_.hardware_parameters["msg_attempts"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.stepper_enc_per_rev = std::stoi(info_.hardware_parameters["stepper_enc_per_rev"]);
    cfg_.stepper_step_per_rev = std::stoi(info_.hardware_parameters["stepper_step_per_rev"]);
    cfg_.bldc_enc_per_rev = std::stoi(info_.hardware_parameters["bldc_enc_per_rev"]);

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        Motor motor;

        if (joint.command_interfaces.size() != 1)
        {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.name.find("arm") != std::string::npos &&
            joint.state_interfaces.size() == 1 &&
            joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
        {
            motor.setup(joint.name, cfg_.stepper_enc_per_rev, cfg_.stepper_step_per_rev);
            motor.pos = 0.0;
            // RCLCPP_INFO(get_logger(), "test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! %f", motor.rads_per_step);
        }
        else if (joint.name.find("wheel") != std::string::npos &&
            joint.state_interfaces.size() == 1 &&
            joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)
        {
            motor.setup(joint.name, cfg_.bldc_enc_per_rev, 0);
            motor.pos = 0.0;
        }
        else if (joint.name.find("servo") != std::string::npos &&
            joint.state_interfaces.size() == 0 &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
        {
            motor.setup(joint.name, 0, 0);
        }
        else
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' is incorrectly configured, check name or interfaces.",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::vector<std::string> state_interface_names;
        for (const auto& state_interface : joint.state_interfaces) 
        {
            state_interface_names.push_back(state_interface.name);
        }

        motors_.emplace_back(motor);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        for (hardware_interface::InterfaceInfo state_interface : info_.joints[i].state_interfaces)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, state_interface.name, &motors_[i].pos));
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &motors_[i].cmd));
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
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.msg_attempts, cfg_.timeout_ms);

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
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!comms_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }

    // std::vector<int> enc_values = comms_.read_encoder_values();

    for (auto i = 0u; i < motors_.size(); i++)
    {
        // if (i > enc_values.size())
        // {
        //     RCLCPP_ERROR(get_logger(), "Not enough encoder values recieved");
        //     return hardware_interface::return_type::ERROR;
        // }

        // if (motors_[i].rads_per_enc != NAN)
        // {
        //     motors_[i].calc_enc_angle(enc_values[i]);
        // }

        motors_[i].calc_enc_angle(motors_[i].cmd*(cfg_.stepper_enc_per_rev/cfg_.stepper_step_per_rev));
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

    std::vector<double> cmd_values(motors_.size());

    for (auto i = 0u; i < motors_.size(); i++)
    {
        if (motors_[i].rads_per_step > 0)
            {
                cmd_values[i] = motors_[i].calc_angle_step();
            }
    }

    comms_.set_motor_values(cmd_values);
    return hardware_interface::return_type::OK;
}
    

}  // namespace rushbott_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rushbott_hardware::RoverSystemHardware, hardware_interface::SystemInterface)