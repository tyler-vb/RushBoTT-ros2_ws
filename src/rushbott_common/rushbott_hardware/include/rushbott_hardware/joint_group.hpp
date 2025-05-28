#ifndef RUSHBOTT_HARDWARE_JOINT_GROUP_HPP
#define RUSHBOTT_HARDWARE_JOINT_GROUP_HPP

#include <vector>
#include <string>
#include <cmath>
#include "rushbott_hardware/motor_packet.hpp"

struct Joint
{
    std::string name = "";
    std::string type = "";
    double cmd_pos = 0.0;
    double cmd_vel = 0.0;
    double state_pos = 0.0;
    double state_vel = 0.0;
};

class JointGroup
{

public:
    JointGroup() = default;

    void add_joint(const std::string name, const std::string type, const double initial_pos)
    {
        Joint joint;
        joint.name = name;
        joint.cmd_pos = initial_pos;
        joint.state_pos = initial_pos;

        if (type == "stepper")
        {
            steppers_.push_back(joint);
        }else if (type == "servo")
        {
            servos_.push_back(joint);
        }
        else if (type == "bldc")
        {
            bldcs_.push_back(joint);
        }
    }

    std::vector<Joint>& get_joints(const std::string type)
    {
        if (type == "stepper")
        {
            return steppers_;
        }
        else if (type == "servo")
        {
            return servos_;
        }
        else if (type == "bldc")
        {
            return bldcs_;
        }
    }

    void import_joint_states(const MotorPacket& packet, double delta_seconds)
    {
        double last_position = steppers_[0].state_pos;
        steppers_[0].state_pos = packet.stepper[0] * stepper_state_conversion_;
        steppers_[0].state_vel = (steppers_[0].state_pos - last_position) / delta_seconds;
        last_position = steppers_[1].state_pos;
        steppers_[1].state_pos = (packet.stepper[1] - packet.stepper[0]) * stepper_state_conversion_;
        steppers_[1].state_vel = (steppers_[1].state_pos - last_position) / delta_seconds;

    }
    
    void export_joint_commands(MotorPacket& packet)
    {
        packet.stepper[0] = static_cast<int16_t>(steppers_[0].cmd_pos * stepper_cmd_conversion_);
        packet.stepper[0] = static_cast<int16_t>(steppers_[0].cmd_vel * stepper_cmd_conversion_);
        packet.stepper[1] = static_cast<int16_t>((steppers_[1].cmd_pos + steppers_[0].cmd_pos) * stepper_cmd_conversion_);
        packet.stepper[1] = static_cast<int16_t>((steppers_[1].cmd_vel + steppers_[0].cmd_vel) * stepper_cmd_conversion_);
    }

    void update_conversion(const std::string type, const double gear_ratio, const double enc_per_rev, const double step_per_rev)
    {
        if (type == "stepper")
        {
            stepper_state_conversion_ = (2*M_PI)/(enc_per_rev*gear_ratio);
            stepper_cmd_conversion_ = (step_per_rev*gear_ratio)/(2*M_PI);
        }
        else if (type == "servo")
        {
            // Currently not used, but can be implemented if needed
        }
        else if (type == "bldc")
        {
            // Currently not used, but can be implemented if needed
        }
    }

    void print_joints()
    {
        for (const auto& joint : steppers_)
        {
            std::cout << "Stepper Joint: " << joint.name 
                      << ", State Position: " << joint.state_pos 
                      << ", Cmd Position: " << joint.cmd_pos << std::endl;
        }
    }

private:
    std::vector<Joint> steppers_;
    std::vector<Joint> servos_;
    std::vector<Joint> bldcs_;

    double stepper_state_conversion_ = 1.0;
    double stepper_cmd_conversion_ = 1.0;
};

#endif // RUSHBOTT_HARDWARE_JOINT_GROUP_HPP
