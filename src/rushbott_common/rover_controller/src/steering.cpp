#include "rover_controller/steering.hpp"

namespace rover_controller
{
SteeringController::SteeringController(
        const std::vector<DriveModule>& drive_modules,
        double vel_limit,
        double angle_limit,
        double wheel_radius,
        const std::function<void(const std::string&)>& logger)
    : modules_(drive_modules),
      vel_limit_(vel_limit),
      angle_limit_(angle_limit),
      logger_(logger),
      wheel_radius_(wheel_radius),
      COT_limit_(set_COT_limit())
{
}

// Calculate the Center of Turning (COT) limit
const double SteeringController::set_COT_limit() 
{
    double limit = 0;
    for (const auto& module : modules_) {
        double current_limit = std::abs(module.x_position / std::tan(angle_limit_) + module.y_position);
        limit = std::max(limit, current_limit);
    }
    limit = std::ceil(limit);
    logger_("Calculated COT limit: " + std::to_string(limit));
    return limit;
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

    logger_("drive velocities: " + std::to_string(drive_velocities.size()));
    logger_("steering angles: " + std::to_string(steering_angles.size()));
}

} // namespace rover_controller
