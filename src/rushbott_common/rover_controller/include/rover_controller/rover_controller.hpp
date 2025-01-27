#ifndef ROVER_CONTROLLER_HPP
#define ROVER_CONTROLLER_HPP

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "rover_controller/steering.hpp"
#include "rover_controller/odometry.hpp"
#include "rover_controller/drive_module.hpp"

namespace rover_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RoverController : public controller_interface::ControllerInterface
{
    using Twist = geometry_msgs::msg::Twist;
    using TwistStamped = geometry_msgs::msg::TwistStamped;
  
public:
    RoverController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
    struct DriveHandle
    {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
    };
    struct SteeringHandle
    {
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command;
    };

    CallbackReturn get_drive_joints(
        const std::string & drive_joint_names, std::vector<DriveHandle> & drive_joints);
    CallbackReturn get_steering_joints(
        const std::string & steering_joint_name, std::vector<SteeringHandle> & steering_joints);

    // Parameters from ROS for rover_controller
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::vector<DriveHandle> drive_joints_;
    std::vector<SteeringHandle> steering_joints_;

    Odometry odometry_;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
        realtime_odometry_publisher_ = nullptr;

    std::chrono::milliseconds cmd_vel_timeout_{500};

    bool subscriber_is_active_ = false;
    rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;
    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_service_;

    bool is_halted = false;

    void reset_odometry(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool reset();
    void halt();
};

}  // namespace tricycle_controller

#endif  // TRICYCLE_CONTROLLER__TRICYCLE_CONTROLLER_HPP_
