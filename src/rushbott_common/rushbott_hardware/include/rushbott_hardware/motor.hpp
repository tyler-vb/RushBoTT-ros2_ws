#ifndef RUSHBOTT_HARDWARE_Motor_HPP
#define RUSHBOTT_HARDWARE_Motor_HPP

#include <string>
#include <cmath>


class Motor
{
    public:

    std::string name = "";
    std::string type = "";
    int enc = 0;
    double cmd = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    double rads_per_count = 0.0;

    Motor() = default;

    Motor(const std::string &motor_name, const auto &cmd_type, double pos_init, double vel_init, int counts_per_rev)
    {
      setup(motor_name, cmd_type, pos_init, vel_init, counts_per_rev);
    }
    
    void setup(const std::string &motor_name, const auto &cmd_type, double pos_init, double vel_init, int counts_per_rev)
    {
      name = motor_name;
      type = cmd_type;
      pos = pos_init;
      vel = vel_init;

      if (counts_per_rev > 0) 
      {
        rads_per_count = (2 * M_PI) / counts_per_rev;
      } 
      else 
      {
        rads_per_count = 0;
      }
    }

    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }
};


#endif // RUSHBOTT_HARDWARE_Motor_HPP