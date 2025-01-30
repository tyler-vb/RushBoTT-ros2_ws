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

#include "rover_controller/odometry.hpp"

namespace rover_controller 
{
Odometry::Odometry(size_t velocity_rolling_window_size)
:   x_(0.0),
    y_(0.0),
    heading_(0.0),
    linear_speed_(0.0),
    angular_speed_(0.0),
    wheel_radius_(0.0),
    modules_({}),
    old_wheel_positions_({}),
    velocity_rolling_window_size_(velocity_rolling_window_size),
    linear_accumulator_(velocity_rolling_window_size),
    angular_accumulator_(velocity_rolling_window_size)
{
}

bool Odometry::update(std::vector<double> wheel_positions, double center_of_turning, const rclcpp::Duration & dt)
{
    std::vector<double> wheel_speeds(old_wheel_positions_.size(), 0.0);

    if (dt.seconds() < 0.0001)
    {
        return false;  // Interval too small to integrate with
    }

    for (size_t i = 0; i < wheel_positions.size(); i++)
    {
        wheel_speeds[i] = (wheel_positions[i]-old_wheel_positions_[i])*dt.seconds();
    }

    old_wheel_positions_ = wheel_positions;

    update_from_velocity(wheel_speeds, center_of_turning, dt);

    return true;
}

void Odometry::update_from_velocity(std::vector<double> wheel_speeds, double center_of_turning, const rclcpp::Duration & dt)
{
    // use wheel speeds and center of turning to find linear and angular body movement
    double body_w = 0.0;
    double body_v = 0.0;

    if (std::isnan(center_of_turning))
    {
        for (auto &wheel_speed : wheel_speeds)
        {
            body_v += wheel_speed;
        }
        body_v /= static_cast<double>(wheel_speeds.size());
    }
    else
    {
        for (size_t i = 0; i < modules_.size(); i++)
        {
            if (!modules_[i].steering_joint_name.empty())
            {
                body_w =+ wheel_speeds[i]*wheel_radius_/(center_of_turning - modules_[i].y_position);
            }
        }
        body_w /= static_cast<double>(wheel_speeds.size());
        body_v = body_w * center_of_turning;
    }
    

    // Integrate odometry:
    integrate_exact(body_v * dt.seconds(), body_w * dt.seconds());

    // Estimate speeds using a rolling mean to filter them out:
    linear_accumulator_.accumulate(body_v);
    angular_accumulator_.accumulate(body_w);

    linear_speed_ = linear_accumulator_.getRollingMean();
    angular_speed_ = angular_accumulator_.getRollingMean();
}

void Odometry::reset_odometry()
{
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
    std::fill(old_wheel_positions_.begin(), old_wheel_positions_.end(), 0.0);
    reset_accumulators();
}

void Odometry::set_wheel_radius(double wheel_radius)
{
    wheel_radius_ = wheel_radius;
}

void Odometry::set_drive_modules(std::vector<DriveModule> & drive_modules)
{
    modules_ = drive_modules;
    
    for (auto &module : modules_)
    {
        if (!module.drive_joint_name.empty())
        {
            old_wheel_positions_.push_back(0.0);
        }
    }
}
void Odometry::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    reset_accumulators();
}

void Odometry::integrate_runge_kutta_2(double ds, double dtheta)
{
    const double direction = heading_ + dtheta * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_ += ds * cos(direction);
    y_ += ds * sin(direction);
    heading_ += dtheta;
}

void Odometry::integrate_exact(double ds, double dtheta)
{
    if (fabs(dtheta) < 1e-6)
    {
        integrate_runge_kutta_2(ds, dtheta);
    }
    else
    {
        /// Exact integration (should solve problems when angular is zero):
        const double heading_old = heading_;
        const double r = ds / dtheta;
        heading_ += dtheta;
        x_ += r * (sin(heading_) - sin(heading_old));
        y_ += -r * (cos(heading_) - cos(heading_old));
    }
}

void Odometry::reset_accumulators()
{
    linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace rover_controller