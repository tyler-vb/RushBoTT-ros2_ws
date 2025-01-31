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
        command_interfaces_config.names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    for (const auto & joint_name : params_.steering_joint_names)
        command_interfaces_config.names.push_back(joint_name + "/" + HW_IF_POSITION);
    return command_interfaces_config;
}

InterfaceConfiguration RoverController::state_interface_configuration() const
{
    InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : params_.drive_joint_names)
        state_interfaces_config.names.push_back(joint_name + "/" + HW_IF_POSITION);
    return state_interfaces_config;
}

controller_interface::return_type RoverController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    auto logger = get_node()->get_logger();

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
            RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
            return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - last_command_msg_->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_)
    {
        last_command_msg_->twist.linear.x = 0.0;
        last_command_msg_->twist.angular.z = 0.0;
    }
    else if (
        !std::isfinite(last_command_msg_->twist.linear.x) ||
        !std::isfinite(last_command_msg_->twist.angular.z))
    {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(
        logger, *get_node()->get_clock(), cmd_vel_timeout_.count(),
        "Command message contains NaNs. Not updating reference interfaces.");
    }

    TwistStamped command = *last_command_msg_;
    double & linear_command = command.twist.linear.x;
    double & angular_command = command.twist.angular.z;
    std::vector<double> wheel_speeds = read_joint_states();

    for (const auto& speed : wheel_speeds) 
    {
        if (std::isnan(speed)) 
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not read wheel state value");
            return controller_interface::return_type::ERROR;
        }
    }

    // update odometry and steering, we update steering first since center of turning is open loop
    steering_controller_.update(linear_command, angular_command, logger);
    odometry_.update(wheel_speeds, steering_controller_.get_COT(), period);

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.get_heading());

    if (realtime_odometry_publisher_->trylock())
        {
            auto & odometry_message = realtime_odometry_publisher_->msg_;
            odometry_message.header.stamp = time;
            if (!params_.odom_only_twist)
            {
            odometry_message.pose.pose.position.x = odometry_.get_X();
            odometry_message.pose.pose.position.y = odometry_.get_Y();
            odometry_message.pose.pose.orientation.x = orientation.x();
            odometry_message.pose.pose.orientation.y = orientation.y();
            odometry_message.pose.pose.orientation.z = orientation.z();
            odometry_message.pose.pose.orientation.w = orientation.w();
            }
            odometry_message.twist.twist.linear.x = odometry_.get_linear_speed();
            odometry_message.twist.twist.angular.z = odometry_.get_angular_speed();
            realtime_odometry_publisher_->unlockAndPublish();
        }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
        auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
        transform.header.stamp = time;
        transform.transform.translation.x = odometry_.get_X();
        transform.transform.translation.y = odometry_.get_Y();
        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();
        transform.transform.rotation.w = orientation.w();
        realtime_odometry_transform_publisher_->unlockAndPublish();
    }

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

    configure_drive_modules();

    steering_controller_.set_drive_modules(drive_modules_);
    steering_controller_.set_limits(params_.vel_limit, params_.angle_limit);
    steering_controller_.set_wheel_radius(params_.wheel_radius);

    odometry_.set_drive_modules(drive_modules_);
    odometry_.set_velocity_rolling_window_size(static_cast<size_t>(params_.velocity_rolling_window_size));
    odometry_.set_wheel_radius(params_.wheel_radius);

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

    // initialize odometry publisher and message
    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        odometry_publisher_);

    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = params_.odom_frame_id;
    odometry_message.child_frame_id = params_.base_frame_id;

    // initialize odom values zeros
    odometry_message.twist =
        geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index)
    {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index = NUM_DIMENSIONS * index + index;
        odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
        odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
    }

    // initialize transform publisher and message
    if (params_.enable_odom_tf)
    {
        odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
        DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
        realtime_odometry_transform_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
            odometry_transform_publisher_);

        // keeping track of odom and base_link transforms only
        auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
        odometry_transform_message.transforms.resize(1);
        odometry_transform_message.transforms.front().header.frame_id = params_.odom_frame_id;
        odometry_transform_message.transforms.front().child_frame_id = params_.base_frame_id;
    }

    // Create odom reset service
    reset_odom_service_ = get_node()->create_service<std_srvs::srv::Empty>(
        DEFAULT_RESET_ODOM_SERVICE, std::bind(
                                    &RoverController::reset_odometry, this, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3));

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_activate(const rclcpp_lifecycle::State &)
{
    for (auto & module : drive_modules_)
    {
        if (module.drive_joint_name.empty() == false)
        {
            const auto result = configure_drive_joint_handle(module.drive_joint_name, registered_drive_handles_);
            if (result == CallbackReturn::ERROR)
            {
                return CallbackReturn::ERROR;
            }
        }
        if (module.steering_joint_name.empty() == false)
        {
            const auto result = configure_steering_joint_handle(module.steering_joint_name, registered_steering_handles_);
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

void RoverController::reset_odometry(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
    odometry_.reset_odometry();
    RCLCPP_INFO(get_node()->get_logger(), "Odometry successfully reset");
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
        (void)registered_drive_handles_[i].command.get().set_value(desired_vels[i]);
    }

    for (size_t i = 0; i < registered_steering_handles_.size(); i++)  
    {  
        (void)registered_steering_handles_[i].command.get().set_value(desired_angles[i]);
    }
}

std::vector<double> RoverController::read_joint_states()
{
    std::vector<double> wheel_speeds(registered_drive_handles_.size(), 0.0);

    for (size_t i = 0; i < registered_drive_handles_.size(); i++)
    {
        wheel_speeds[i] = registered_drive_handles_[i].state.get().get_value();
    }

    return wheel_speeds;
}

void RoverController::configure_drive_modules()
{
    const double track_width = params_.track_width;
    const double wheelbase = params_.wheelbase;
    const std::vector<std::string> steering_joint_names = params_.steering_joint_names;
    const std::vector<std::string> drive_joint_names = params_.drive_joint_names;

    // acceptable rover module names
    std::vector<std::string> rover_module_names = {"front_left", "front_right", "mid_left", "mid_right", "rear_left", "rear_right"};

    for (size_t i = 0; i < rover_module_names.size(); i++)
    {
        DriveModule module;

        bool include_steering_joint = true;
        bool include_drive_joint = true;

        if (rover_module_names[i] == "front_left")
        {
            module.y_position = track_width / 2;
            module.x_position = wheelbase / 2;
        }
        else if (rover_module_names[i] == "front_right")
        {
            module.y_position = -track_width / 2;
            module.x_position = wheelbase / 2;
        }
        else if (rover_module_names[i] == "mid_left")
        {
            module.y_position = track_width / 2;
            module.x_position = 0;
            include_steering_joint = false;
        }
        else if (rover_module_names[i] == "mid_right")
        {
            module.y_position = -track_width / 2;
            module.x_position = 0;
            include_steering_joint = false;
        }
        else if (rover_module_names[i] == "rear_left")
        {
            module.y_position = track_width / 2;
            module.x_position = -wheelbase / 2;
        }
        else if (rover_module_names[i] == "rear_right")
        {
            module.y_position = -track_width / 2;
            module.x_position = -wheelbase / 2;
        }

        if (include_steering_joint)
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

        if (include_drive_joint)
        {
            const auto drive_joint_name = std::find_if(
            drive_joint_names.cbegin(), drive_joint_names.cend(),
            [&](const std::string& name)
            {
                return name.find(rover_module_names[i]) != std::string::npos;
            });

            if (drive_joint_name != drive_joint_names.cend())
            {
                module.drive_joint_name = *drive_joint_name;
                module.name = rover_module_names[i];
            }             
        }  

        if (module.name.empty() == false) 
        {
            drive_modules_.push_back(module);
        }
    }
}

CallbackReturn RoverController::configure_drive_joint_handle(
    const std::string & joint_name, std::vector<DriveJointHandle> & registered_handles)
{
    auto logger = get_node()->get_logger();

    // lookup velocity command interface
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto & interface)
        {
            return interface.get_prefix_name() == joint_name &&
            interface.get_interface_name() == HW_IF_VELOCITY;
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
        DriveJointHandle{std::ref(*state_handle), std::ref(*command_handle)});
    return CallbackReturn::SUCCESS;
} 

CallbackReturn RoverController::configure_steering_joint_handle(
    const std::string & joint_name, std::vector<SteeringJointHandle> & registered_handles)
{
    auto logger = get_node()->get_logger();

    // lookup velocity command interface
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto & interface)
        {   
            return interface.get_prefix_name() == joint_name &&
            interface.get_interface_name() == HW_IF_POSITION;            
        });
        
    if (command_handle == command_interfaces_.end())
    {
        RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
        SteeringJointHandle{std::ref(*command_handle)});
    return CallbackReturn::SUCCESS;
} 
        

}  // namespace rover_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rover_controller::RoverController, controller_interface::ControllerInterface)