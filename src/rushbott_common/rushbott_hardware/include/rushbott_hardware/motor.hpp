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
    double pos = 0.0;
    double vel = 0.0;
    double rads_per_count = 0.0;

    Motor() = default;

    Motor(const std::string &joint_name, int counts_per_rev)
    {
      setup(joint_name, counts_per_rev);
    }
    
    void setup(const std::string &joint_name, int counts_per_rev)
    {
      name = joint_name;

      if (counts_per_rev > 0) 
      {
        rads_per_count = (2 * M_PI) / counts_per_rev;
      } 
      else 
      {
        rads_per_count = 0;
      }
    }

    void calc_enc_angle(int enc)
    {
      pos = enc * rads_per_count;
    }
};


#endif // RUSHBOTT_HARDWARE_Motor_HPP