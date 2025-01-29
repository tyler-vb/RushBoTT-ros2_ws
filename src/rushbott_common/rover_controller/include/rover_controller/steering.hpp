#ifndef STEERING_CONTROLLER_HPP
#define STEERING_CONTROLLER_HPP

#include <cmath>
#include <functional>
#include <vector>
#include <string>
#include <algorithm>

#include "rover_controller/drive_module.hpp"

namespace rover_controller
{
class SteeringController 
{
public:
    // Constructor
    SteeringController(
        const std::vector<DriveModule>& drive_modules,
        const double vel_limit,
        const double angle_limit,
        const double wheel_radius,
        const std::function<void(const std::string&)>& logger);

    std::vector<double> get_desired_vels() const { return desired_vels_; }
    std::vector<double> get_desired_angles() const { return desired_angles_; }

    // Update drive module states based on body velocity and angular velocity
    void update(double body_v, double body_w);

private:

    // Calculate the COT (Center of Turning) limit
    const double set_COT_limit();

    const std::vector<DriveModule> modules_;         // List of drive modules
    const double vel_limit_;                         // Velocity limit
    const double angle_limit_;                       // Steering angle limit
    const double wheel_radius_;
    const std::function<void(const std::string&)> logger_; // Logger function

    std::vector<double> desired_vels_;
    std::vector<double> desired_angles_;
    const double COT_limit_;                           // Center of Turning limit
};

} // namespace rover_controller 

#endif // STEERING_CONTROLLER_HPP