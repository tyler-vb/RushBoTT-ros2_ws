#ifndef ROVER_CONTROLLER__ROVER_CONTROLLER_HPP_
#define ROVER_CONTROLLER__ROVER_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "rover_controller/rover_controller_parameters.hpp"

namespace rover_controller
{
class RoverController : public controller_interface::ControllerInterface
{
  using TwistStamped = geometry_msgs::msg::TwistStamped;

public:
  RoverController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:

  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  // configure wheels on each side, change to configuring drive modules
  controller_interface::CallbackReturn configure_side(
    const std::string & side, 
    const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles);

  // registered wheels (change this to drive modules later)
  std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_right_wheel_handles_;

  bool subscriber_is_active_ = false;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistStamped>> received_velocity_msg_ptr_{nullptr};

  std::queue<std::array<double, 2>> previous_two_commands_;
  // speed limiters
  std::unique_ptr<SpeedLimiter> limiter_linear_;
  std::unique_ptr<SpeedLimiter> limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<TwistStamped>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<TwistStamped>>
    realtime_limited_velocity_publisher_ = nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  bool is_halted = false;

  bool reset();
  void halt();

private:
  void reset_buffers();
  };
  }  // namespace rover_controller
#endif  // ROVER_CONTROLLER__ROVER_CONTROLLER_HPP_
