#ifndef DRIVE_MODULE_HPP
#define DRIVE_MODULE_HPP

#include <string>

namespace rover_controller
{
struct DriveModule 
{
    // default empy values
    std::string name;         
    std::string steering_joint_name;        
    std::string drive_joint_name;          
    double x_position;         
    double y_position;         
};

} // namespace

#endif // DRIVE_MODULE_HPP