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
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.step_enc_counts_per_rev = std::stoi(info_.hardware_parameters["step_enc_counts_per_rev"]);
    cfg_.bldc_enc_counts_per_rev = std::stoi(info_.hardware_parameters["bldc_enc_counts_per_rev"]);

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
            joint.state_interfaces.size() != 1 &&
            joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
        {
            motor.setup(joint.name, hardware_interface::HW_IF_POSITION, 0, NAN, cfg_.step_enc_counts_per_rev);
        }
        else if (joint.name.find("wheel") != std::string::npos &&
            joint.state_interfaces.size() == 1 &&
            joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)
        {
            motor.setup(joint.name, hardware_interface::HW_IF_VELOCITY, 0, NAN, cfg_.bldc_enc_counts_per_rev);
        }
        else if (joint.name.find("servo") != std::string::npos &&
            joint.state_interfaces.size() == 0 &&
            joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
        {
            motor.setup(joint.name, hardware_interface::HW_IF_POSITION, NAN, NAN, 0);
        }
        else
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' is incorrectly configured, check name or interfaces.",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        motors_.emplace_back(motor);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto i = 0u; i < motors_.size(); i++)
    {
        if (motors_[i].pos != NAN)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                motors_[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].pos));
        }
        if (motors_[i].vel != NAN)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                motors_[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].vel));
        }
    }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < motors_.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        motors_[i].name, motors_[i].type, &motors_[i].cmd));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RoverSystemHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring rover interfaces");


  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating rover interfaces");

    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

    // command and state should be equal when starting
    for (const auto & [name, descr] : joint_command_interfaces_)
    {
        set_command(name, get_state(name));
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating rover interfaces");
    comms_.disconnect();
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverSystemHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    std::stringstream ss;
    ss << "Reading states:";
    ss << std::fixed << std::setprecision(2);
    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        auto pos = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_POSITION);
        set_state(name, get_state(name));
        if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
        {
        // Simulate DiffBot wheels's movement as a first-order system
        // Update the joint status: this is a revolute joint without any limit.
        // Simply integrates
        auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
        set_state(name, get_state(name) + period.seconds() * velo);

        ss << std::endl
            << "\t position " << get_state(name) << " and velocity " << velo << " for '" << name
            << "'!";
        }
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type rushbott_hardware::RoverSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    std::stringstream ss;
    ss << "Writing commands:";
    for (const auto & [name, descr] : joint_command_interfaces_)
    {
        // Simulate sending commands to the hardware
        set_state(name, get_command(name));

        ss << std::fixed << std::setprecision(2) << std::endl
        << "\t" << "command " << get_command(name) << " for '" << name << "'!";
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
}

}  // namespace rushbott_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rushbott_hardware::RoverSystemHardware, hardware_interface::SystemInterface)