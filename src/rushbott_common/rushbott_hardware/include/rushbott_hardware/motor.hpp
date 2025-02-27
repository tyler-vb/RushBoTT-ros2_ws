#ifndef RUSHBOTT_HARDWARE_Motor_HPP
#define RUSHBOTT_HARDWARE_Motor_HPP

#include <string>
#include <vector>
#include <cmath>


class Motor
{
    public:

    std::string name = "";
    double cmd = 0.0;
    double pos = NAN;
    double vel = NAN;
    double rads_per_enc = 0;
    double rads_per_step = 0;

    Motor() = default;

    Motor(const std::string &joint_name, int enc_per_rev, int step_per_rev)
    {
      setup(joint_name, enc_per_rev, step_per_rev);
    }
    
    void setup(const std::string &joint_name, int enc_per_rev, int step_per_rev)
    {
      name = joint_name;

      if (enc_per_rev > 0) 
      {
        rads_per_enc = (2 * M_PI) / enc_per_rev;
      } 

      if (step_per_rev > 0) 
      {
        rads_per_step = (2 * M_PI) / step_per_rev;
      } 

    }

    void calc_enc_angle(int enc)
    {
      pos = enc * rads_per_enc;
    }

    int calc_angle_step()
    {
      int step = static_cast<int>(cmd / rads_per_step);
      return step;
    }
};


#endif // RUSHBOTT_HARDWARE_Motor_HPP