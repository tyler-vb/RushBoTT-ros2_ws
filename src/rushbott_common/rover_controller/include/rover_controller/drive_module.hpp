#ifndef DRIVE_MODULE_HPP
#define DRIVE_MODULE_HPP

#include <string>

namespace rover_controller
{
struct DriveModule {
    std::string name;          // Name of the module
    int steering_index;        // Steering index
    int drive_index;           // Drive index
    double x_position;         // X position
    double y_position;         // Y position

    DriveModule(
        const std::string &name,
        int steering_index,
        int drive_index,
        double x_position,
        double y_position
    )
        : name(name), steering_index(steering_index), drive_index(drive_index),
          x_position(x_position), y_position(y_position) {}
};

} // namespace

#endif // DRIVE_MODULE_HPP