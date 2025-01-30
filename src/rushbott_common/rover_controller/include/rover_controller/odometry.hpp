#ifndef ROVER_CONTROLLER__ODOMETRY_HPP_
#define ROVER_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include <rclcpp/duration.hpp>
#include "rcpputils/rolling_mean_accumulator.hpp"

#include "rover_controller/drive_module.hpp"

namespace rover_controller
{
class Odometry
{
    using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
    
    public:
        explicit Odometry(size_t velocity_rolling_window_size = 10);

        bool update(std::vector<double> wheel_positions, double center_of_turning, const rclcpp::Duration & dt);
        void update_from_velocity(std::vector<double> wheel_speeds, double center_of_turning, const rclcpp::Duration & dt);
        void reset_odometry();

        double get_X() const { return x_; }
        double get_Y() const { return y_; }
        double get_heading() const { return heading_; }
        double get_linear_speed() const { return linear_speed_; }
        double get_angular_speed() const { return angular_speed_; }

        void set_wheel_radius(double wheel_radius);
        void set_drive_modules(std::vector<DriveModule> & drive_modules);
        void set_velocity_rolling_window_size(size_t velocity_rolling_window_size);

    private:
        void integrate_runge_kutta_2(double ds, double dtheta);
        void integrate_exact(double ds, double dtheta);
        void reset_accumulators();

        // Current pose:
        double x_;        //   [m]
        double y_;        //   [m]
        double heading_;  // [rad]

        // Current velocity:
        double linear_speed_;   //   [m/s]
        double angular_speed_;  // [rad/s]

        // Wheel kinematic parameters [m]:
        double wheel_radius_;

        std::vector<DriveModule> modules_;
        std::vector<double> old_wheel_positions_;

        // Rolling mean accumulators for the linear and angular velocities:
        size_t velocity_rolling_window_size_;
        RollingMeanAccumulator linear_accumulator_;
        RollingMeanAccumulator angular_accumulator_;
};

}  // namespace rover_controller

#endif  // ROVER_CONTROLLER__ODOMETRY_HPP_