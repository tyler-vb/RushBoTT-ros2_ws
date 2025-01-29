#include "rover_controller/steering.hpp"

namespace rover_controller
{
SteeringController::SteeringController()
: modules_({}),
    vel_limit_(0.0),
    angle_limit_(0.0),
    wheel_radius_(0.0),
    COT_limit_(0.0)
{
}

void SteeringController::set_limits(double vel_limit, double angle_limit)
{
    vel_limit_ = vel_limit;
    angle_limit_ = angle_limit;
    set_COT_limit();
}
void SteeringController::set_wheel_radius(double wheel_radius)
{
    wheel_radius_ = wheel_radius;
}
void SteeringController::set_drive_modules(std::vector<DriveModule> & drive_modules)
{
    modules_ = drive_modules;
}
void SteeringController::set_COT_limit() 
{
    for (const auto& module : modules_) {
        double current_limit = std::abs(module.x_position / std::tan(angle_limit_) + module.y_position);
        COT_limit_ = std::max(COT_limit_, current_limit);
    }
}

// Update drive module states
void SteeringController::update(double body_v, double body_w) 
{
    std::vector<double> drive_velocities;
    std::vector<double> steering_angles;

    double scale = 1.0;

    for (const auto& module : modules_) 
    {
        double drive_velocity = 0.0;
        double steering_angle = 0.0;

        if (std::abs(body_v) < 1e-6) 
        {
            // Skip when body_v is effectively zero
            continue;
        }

        else if (std::abs(body_w) < 1e-6) 
        {
            drive_velocity = body_v / wheel_radius_;
            scale = std::min(vel_limit_ / std::abs(drive_velocity), scale);

        } 
        else 
        {
            double center_of_turning = std::clamp(body_v/body_w,-COT_limit_,COT_limit_);
            body_w = body_v/center_of_turning;

            double distance_from_COT = std::sqrt(
                std::pow(center_of_turning - module.y_position, 2) +
                std::pow(-module.x_position, 2)
            );

            drive_velocity = std::copysign(distance_from_COT * body_w / wheel_radius_, body_v);
            steering_angle = std::atan(-module.x_position / (center_of_turning - module.y_position));

            scale = std::min(vel_limit_ / std::abs(drive_velocity), scale);
        }

        if (module.drive_joint_name.empty() == false) 
        {
            drive_velocities.push_back(drive_velocity);
        }

        if (module.steering_joint_name.empty() == false) 
        {
            steering_angles.push_back(steering_angle);
        }
    }

    for (auto& velocity : drive_velocities) 
    {
        velocity *= scale;
    }

    desired_vels_ = drive_velocities;
    desired_angles_ = steering_angles;
}

} // namespace rover_controller
