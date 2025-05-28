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

    RCLCPP_INFO(get_logger(), "Initializing...");

    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

    cfg_.stepper_gear_ratio = std::stod(info_.hardware_parameters["stepper_gear_ratio"]);
    cfg_.stepper_enc_per_rev = std::stoi(info_.hardware_parameters["stepper_enc_per_rev"]);
    cfg_.stepper_step_per_rev = std::stoi(info_.hardware_parameters["stepper_step_per_rev"]);

    cfg_.bldc_enc_per_rev = std::stoi(info_.hardware_parameters["bldc_enc_per_rev"]);

    joint_group_.update_conversion("stepper", cfg_.stepper_gear_ratio, cfg_.stepper_enc_per_rev, cfg_.stepper_step_per_rev);

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        double initial_value = 0.0;
        double state_conversion = 1.0;
        double cmd_conversion = 1.0;
        std::string type = "";

        if (joint.name.find("arm") != std::string::npos /*&&*/
            // joint.state_interfaces.size() == 2 &&
            // joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            // joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY &&
            // joint.command_interfaces.size() == 2 &&
            // joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            // joint.command_interfaces[1].name == hardware_interface::HW_IF_VELOCITY
        )
        {
            initial_value = std::stod(joint.parameters.at("initial_value"));
            type = "stepper";
        }
        else if (joint.name.find("wheel") != std::string::npos &&
            joint.state_interfaces.size() == 1 &&
            joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY
        )
        {
        }
        else if (joint.name.find("servo") != std::string::npos &&
            joint.state_interfaces.size() == 0 &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION
        )
        {
        }
        else
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' is incorrectly configured, check name or interfaces.",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        joint_group_.add_joint(joint.name, type, initial_value);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto & joint : joint_group_.get_joints("stepper"))
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_POSITION, &joint.state_pos));
        // state_interfaces.emplace_back(hardware_interface::StateInterface(
        //     joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state_vel));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto & joint : joint_group_.get_joints("stepper"))
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_POSITION, &joint.cmd_pos));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //     joint.name, hardware_interface::HW_IF_VELOCITY, &joint.cmd_vel));
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

    if (!comms_.connect(packet_, cfg_.device, cfg_.baud_rate, cfg_.timeout_ms))
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

    calibrating_ = true;

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

    if (comms_.read_encoders(packet_))
    {
        joint_group_.import_joint_states(packet_, delta_seconds);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type rushbott_hardware::RoverSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    if (!comms_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }

    joint_group_.export_joint_commands(packet_);
    
    comms_.set_motors(packet_, calibrating_);

    return hardware_interface::return_type::OK;
}
    

}  // namespace rushbott_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rushbott_hardware::RoverSystemHardware, hardware_interface::SystemInterface)