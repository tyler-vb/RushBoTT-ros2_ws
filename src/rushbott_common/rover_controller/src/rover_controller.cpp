// Copyright 2022 Pixel Robotics.
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

/*
 * Author: Tony Najjar
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "rover_controller/rover_controller.hpp"

namespace rover_controller
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
constexpr auto DEFAULT_RESET_ODOM_SERVICE = "~/reset_odometry";


using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

RoverController::RoverController() : controller_interface::ControllerInterface() {}

CallbackReturn RoverController::on_init()
{
    try
    {
    // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

InterfaceConfiguration RoverController::command_interface_configuration() const
{
    InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : params_.drive_joint_names)
        command_interfaces_config.names.push_back(params_.joint_name + "/" + HW_IF_VELOCITY);
    for (const auto & joint_name : params_.steering_joint_names)
        command_interfaces_config.names.push_back(params_.joint_name + "/" + HW_IF_POSITION);
    return command_interfaces_config;
}

InterfaceConfiguration RoverController::state_interface_configuration() const
{
    InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : params_.drive_joint_names)
        state_interfaces_config.names.push_back(params_.joint_name + "/" + HW_IF_POSITION);
    return state_interfaces_config;
}

controller_interface::return_type RoverController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
    {
            if (!is_halted)
            {
                halt();
                is_halted = true;
            }
            return controller_interface::return_type::OK;
    }

    // if the mutex is unable to lock, last_command_msg_ won't be updated
    received_velocity_msg_ptr_.try_get([this](const std::shared_ptr<TwistStamped> & msg)
                                        { last_command_msg_ = msg; });
    if (last_command_msg_ == nullptr)
    {
            RCLCPP_WARN(get_node()->get_logger(), "Velocity message received was a nullptr.");
            return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - last_command_msg_->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_)
    {
        last_command_msg_->twist.linear.x = 0.0;
        last_command_msg_->twist.angular.z = 0.0;
    }

    // command may be limited further by Limiters,
    // without affecting the stored twist command
    TwistStamped command = *last_command_msg_;
    double & linear_command = command.twist.linear.x;
    double & angular_command = command.twist.angular.z;

    // update steering controller
    steering_controller_.update(linear_command, angular_command);

    write_to_joints(steering_controller_.get_desired_vels(), steering_controller_.get_desired_angles());

    return controller_interface::return_type::OK;
}

CallbackReturn RoverController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto logger = get_node()->get_logger();

    // update parameters if they have changed
    if (param_listener_->is_old(params_))
    {
        params_ = param_listener_->get_params();
        RCLCPP_INFO(logger, "Parameters were updated");
    }

    cmd_vel_timeout_ = std::chrono::milliseconds{params_.cmd_vel_timeout};

    if (!reset())
    {
        return CallbackReturn::ERROR;
    }

    last_command_msg_ = std::make_shared<TwistStamped>();
    received_velocity_msg_ptr_.set([this](std::shared_ptr<TwistStamped> & stored_value)
                                    { stored_value = last_command_msg_; });

  // initialize command subscriber
    velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<TwistStamped> msg) -> void
        {
        if (!subscriber_is_active_)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
            RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
            msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set([msg](std::shared_ptr<TwistStamped> & stored_value)
                                        { stored_value = std::move(msg); });
        });


    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "On activate: Initialize Joints");

    for (auto & module : drive_modules_)
    {
        if (module.drive_joint_name.empty() == false)
        {
            const auto result = configure_joint_handle(module.drive_joint_name, registered_drive_handles_, HW_IF_VELOCITY);
            if (result == CallbackReturn::ERROR)
            {
                return CallbackReturn::ERROR;
            }
        }
        if (module.steering_joint_name.empty() == false)
        {
            const auto result = configure_joint_handle(module.steering_joint_name, registered_steering_handles_, HW_IF_POSITION);
            if (result == CallbackReturn::ERROR)
            {
                return CallbackReturn::ERROR;
            }
        }
    }
            
    if (registered_drive_handles_.empty() && registered_steering_handles_.empty())
    {
        RCLCPP_ERROR(
        get_node()->get_logger(), "No joints were configured, check your configuration");
        return CallbackReturn::ERROR;
    }

    is_halted = false;
    subscriber_is_active_ = true;

    RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_deactivate(const rclcpp_lifecycle::State &)
{
    subscriber_is_active_ = false;
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_cleanup(const rclcpp_lifecycle::State &)
{
    if (!reset())
    {
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_error(const rclcpp_lifecycle::State &)
{
    if (!reset())
    {
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

bool RoverController::reset()
{

    registered_drive_handles_.clear();
    registered_steering_handles_.clear();

    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();

    received_velocity_msg_ptr_.set(nullptr);
    is_halted = false;
    return true;
}

void RoverController::halt()
{
    const std::vector<double> desired_angles(registered_steering_handles_.size(), 0.0);
    const std::vector<double> desired_vels(registered_steering_handles_.size(), 0.0);

    write_to_joints(desired_vels, desired_angles);
}

void RoverController::write_to_joints(
    const std::vector<double> &desired_vels, const std::vector<double> &desired_angles)
{

    for (size_t i = 0; i < registered_drive_handles_.size(); i++)
    {
        registered_drive_handles_[i].command.get().set_value(desired_vels[i]);
    }

    for (size_t i = 0; i < registered_steering_handles_.size(); i++)  
    {  
        registered_steering_handles_[i].command.get().set_value(desired_angles[i]);
    }
}

void RoverController::configure_drive_modules()
{
    const float & track_width = params_.track_width;
    const float & wheelbase = params_.wheelbase;
    const std::vector<std::string> & steering_joint_names = params_.steering_joint_names;
    const std::vector<std::string> & drive_joint_names = params_.drive_joint_names;

    // acceptable rover module names
    std::vector<std::string> rover_module_names = {"front_left", "front_right", "mid_left", "mid_right", "rear_left", "rear_right"};

    for (size_t i = 0; i < rover_module_names.size(); i++)
    {
        DriveModule module;

        bool include_steering_joint = true;
        bool include_drive_joint = true;

        if (rover_module_names[i] == "front_left")
        {
            module.x_position = track_width / 2;
            module.y_position = wheelbase / 2;
        }
        else if (rover_module_names[i] == "front_right")
        {
            module.x_position = -track_width / 2;
            module.y_position = wheelbase / 2;
        }
        else if (rover_module_names[i] == "mid_left")
        {
            module.x_position = track_width / 2;
            module.y_position = 0;
            include_steering_joint = false;
        }
        else if (rover_module_names[i] == "mid_right")
        {
            module.x_position = -track_width / 2;
            module.y_position = 0;
            include_steering_joint = false;
        }
        else if (rover_module_names[i] == "rear_left")
        {
            module.x_position = track_width / 2;
            module.y_position = -wheelbase / 2;
        }
        else if (rover_module_names[i] == "rear_right")
        {
            module.x_position = -track_width / 2;
            module.y_position = -wheelbase / 2;
        }

        if (include_steering_joint = true)
        {
            const auto steering_joint_name = std::find_if(
            steering_joint_names.cbegin(), steering_joint_names.cend(),
            [&](const std::string& name)
            {
                return name.find(rover_module_names[i]) != std::string::npos;
            });

            if (steering_joint_name != steering_joint_names.cend())
            {
                module.steering_joint_name = *steering_joint_name;
                module.name = rover_module_names[i];
            } 
        }

        if (include_drive_joint = true)
        {
            const auto drive_joint_name = std::find_if(
            drive_joint_names.cbegin(), drive_joint_names.cend(),
            [&](const std::string& name)
            {
                return name.find(rover_module_names[i]) != std::string::npos;
            });

            if (drive_joint_name != drive_joint_names.cend())
            {
                module.steering_joint_name = *drive_joint_name;
                module.name = rover_module_names[i];
            }             
        }  

        if (module.name.empty() == false) 
        {
            drive_modules_.push_back(module);
        }
    }
}

CallbackReturn RoverController::configure_joint_handle(
    const std::string & joint_name, std::vector<JointHandle> & registered_handles, const char * command_interface_type)
{
    auto logger = get_node()->get_logger();

    // lookup velocity command interface
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto & interface)
        {
            return interface.get_prefix_name() == joint_name &&
                interface.get_interface_name() == command_interface_type;
        });
        
    if (command_handle == command_interfaces_.end())
    {
        RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

            // lookup position state interface
    const auto state_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&joint_name](const auto & interface)
        {
            return interface.get_prefix_name() == joint_name &&
                interface.get_interface_name() == HW_IF_POSITION;
        });
        
    if (state_handle == state_interfaces_.cend())
    {
        RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
        JointHandle{std::ref(*state_handle), std::ref(*command_handle)});

    }
    

}  // namespace rover_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tricycle_controller::RoverController, controller_interface::ControllerInterface)