#ifndef STEERING_CONTROLLER_HPP
#define STEERING_CONTROLLER_HPP

#include <cmath>
#include <functional>
#include <vector>
#include <string>

#include <rclcpp/duration.hpp>
#include "rover_controller/drive_module.hpp"
#include "rclcpp/logging.hpp"

namespace rover_controller
{
class SteeringController 
{
    public:
        // Constructor
        explicit SteeringController();

        std::vector<double> get_desired_vels() const { return desired_vels_; }
        std::vector<double> get_desired_angles() const { return desired_angles_; }

        // Update drive module states based on body velocity and angular velocity
        void update(double body_v, double body_w, const rclcpp::Logger& logger);

        void set_limits(double vel_limit, double angle_limit);
        void set_wheel_radius(double wheel_radius);
        void set_drive_modules(std::vector<DriveModule> &);
        void set_rise_time(double rise_time);

        double get_COT() const { return center_of_turning_; }

    private:
        // Calculate the COT (Center of Turning) limit
        void set_COT_limit();

        std::vector<DriveModule> modules_;         // List of drive modules
        double vel_limit_;                         // Velocity limit
        double angle_limit_;                       // Steering angle limit
        double wheel_radius_;

        std::vector<double> desired_vels_;
        std::vector<double> desired_angles_;
        double center_of_turning_;
        double COT_limit_;                           // Center of Turning limit
};

} // namespace rover_controller 

#endif // STEERING_CONTROLLER_HPP