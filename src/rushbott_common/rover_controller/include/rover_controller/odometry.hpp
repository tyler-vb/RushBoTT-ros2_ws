#ifndef ROVER_CONTROLLER__ODOMETRY_HPP_
#define ROVER_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include <rclcpp/duration.hpp>
#include "rcpputils/rolling_mean_accumulator.hpp"

namespace rover_controller
{
class Odometry
{
    using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
    
    public:
        explicit Odometry(size_t velocity_rolling_window_size = 10);

        bool update(double left_vel, double right_vel, const rclcpp::Duration & dt);
        void updateOpenLoop(double linear, double angular, const rclcpp::Duration & dt);
        void resetOdometry();

        double getX() const { return x_; }
        double getY() const { return y_; }
        double getHeading() const { return heading_; }
        double getLinear() const { return linear_; }
        double getAngular() const { return angular_; }

        void setWheelParams(double wheel_separation, double wheel_radius);
        void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

    private:
        void integrateRungeKutta2(double linear, double angular);
        void integrateExact(double linear, double angular);
        void resetAccumulators();

        // Current pose:
        double x_;        //   [m]
        double y_;        //   [m]
        double heading_;  // [rad]

        // Current velocity:
        double linear_;   //   [m/s]
        double angular_;  // [rad/s]

        // Wheel kinematic parameters [m]:
        double wheelbase_;
        double wheel_radius_;

        // Rolling mean accumulators for the linear and angular velocities:
        size_t velocity_rolling_window_size_;
        RollingMeanAccumulator linear_accumulator_;
        RollingMeanAccumulator angular_accumulator_;
};

}  // namespace rover_controller

#endif  // ROVER_CONTROLLER__ODOMETRY_HPP_