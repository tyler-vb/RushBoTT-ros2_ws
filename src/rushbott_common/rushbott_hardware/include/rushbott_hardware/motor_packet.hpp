#ifndef RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP
#define RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP
 
#include <vector>

struct __attribute__((packed)) MotorPacket
{
  uint8_t header = 0x64;
  uint8_t flag;
  int8_t bldc[6];
  int16_t servo[4];
  int16_t stepper_pos[2]; 
  int16_t stepper_vel[2];
  uint8_t checksum;

  enum MotorType { BLDC, SERVO, STEPPER };

  enum Flag { ACK = 0xA1, NACK = 0xA2, ENC = 0xB1, MOT = 0xB2, HEY = 0xC1, CAL = 0xD1 };

  uint8_t calculate_checksum() const
  {
    uint8_t sum = header + flag;
    for (size_t i = 0; i < sizeof(bldc) / sizeof(bldc[0]); ++i)
    {
      sum += static_cast<uint8_t>(bldc[i]);
    }
    for (size_t i = 0; i < sizeof(servo) / sizeof(servo[0]); ++i) 
    {
      sum += static_cast<uint8_t>(servo[i] & 0xFF);     
      sum += static_cast<uint8_t>((servo[i] >> 8) & 0xFF);
    }
    for (size_t i = 0; i < sizeof(stepper_pos) / sizeof(stepper_pos[0]); ++i) 
    {
      sum += static_cast<uint8_t>(stepper_pos[i] & 0xFF);      
      sum += static_cast<uint8_t>((stepper_pos[i] >> 8) & 0xFF);
    }
    for (size_t i = 0; i < sizeof(stepper_vel) / sizeof(stepper_vel[0]); ++i) 
    {
      sum += static_cast<uint8_t>(stepper_vel[i] & 0xFF);      
      sum += static_cast<uint8_t>((stepper_vel[i] >> 8) & 0xFF);
    }
    return sum;
  }

  void print_packet(std::string title) const
  {
    std::cout << "[INFO] " << title << "\n";

    std::cout << "  Header: 0x" << std::hex << static_cast<int>(header) << std::dec << "\n";
    std::cout << "  Flag: 0x" << std::hex << static_cast<int>(flag) << std::dec << "\n";

    std::cout << "  BLDC: ";
    for (size_t i = 0; i < std::size(bldc); i++)
    {
        std::cout << static_cast<int>(bldc[i]) << " ";
    }
    std::cout << "\n";

    std::cout << "  Servo: ";
    for (size_t i = 0; i < std::size(servo); i++)
    {
        std::cout << servo[i] << " ";
    }
    std::cout << "\n";

    std::cout << "  Stepper Pos: ";
    for (size_t i = 0; i < std::size(stepper_pos); i++)
    {
        std::cout << stepper_pos[i] << " ";
    }
    std::cout << "\n";

    std::cout << "  Stepper Vel: ";
    for (size_t i = 0; i < std::size(stepper_vel); i++)
    {
        std::cout << stepper_vel[i] << " ";
    }
    std::cout << "\n";

    std::cout << "  Checksum: 0x" << std::hex << static_cast<int>(checksum) << std::dec << "\n";
  }
};




#endif // RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP