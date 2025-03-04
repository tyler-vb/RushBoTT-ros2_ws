#ifndef RUSHBOTT_HARDWARE_Motor_HPP
#define RUSHBOTT_HARDWARE_Motor_HPP

#include <string>
#include <vector>
#include <cmath>


class Stepper
{
    public:

    std::string name = "";
    double cmd = 0.0;
    double pos = 0.0;
    double rads_per_enc = 0;
    double rads_per_step = 0;
    int gear_ratio = 0;

    Stepper() = default;

    Stepper(const std::string &joint_name, int enc_per_rev, int step_per_rev, int ratio)
    {
      setup(joint_name, enc_per_rev, step_per_rev, ratio);
    }
    
    void setup(const std::string &joint_name, int enc_per_rev, int step_per_rev, int ratio)
    {
      name = joint_name;
      gear_ratio = ratio; 

      rads_per_enc = (2 * M_PI) / enc_per_rev;
      rads_per_step = (2 * M_PI) / step_per_rev;
    }

    void calc_angle_from_enc(int enc)
    {
      pos = enc * rads_per_enc / gear_ratio;
    }

    int calc_step_from_angle()
    {
      return static_cast<int>(gear_ratio * (cmd / rads_per_step));
    }
};


#endif // RUSHBOTT_HARDWARE_Motor_HPP