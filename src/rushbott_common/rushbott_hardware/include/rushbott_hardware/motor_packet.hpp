#ifndef RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP
#define RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP
 
#include <vector>

struct __attribute__((packed)) MotorPacket
{
  uint8_t header = 0x00;
  uint8_t flag;
  uint8_t bldc[6];
  uint16_t servo[4];
  uint16_t stepper[2]; 
  uint8_t checksum;

  enum MotorType { BLDC, SERVO, STEPPER };

  enum Flag { ACK = 0xA1, NACK = 0xA2, ENC = 0xB1, MOT = 0xB1, HEY = 0xC1 };

  uint8_t calculate_checksum() const
  {
      uint8_t sum = header + flag;
      for (size_t i = 0; i < sizeof(bldc) / sizeof(bldc[0]); ++i)
      {
        sum += bldc[i];
      }
      for (size_t i = 0; i < sizeof(servo) / sizeof(servo[0]); ++i) 
      {
          sum += servo[i];
      }
      for (size_t i = 0; i < sizeof(stepper) / sizeof(stepper[0]); ++i) {
          sum += stepper[i];
      }

      return sum;
  }

  void import_commands(MotorType type, const std::vector<double> &values, double conversion, std::vector<double> offsets = {}) 
  {
    size_t count = 0;

    switch (type)
    {
        case BLDC: 
          count = std::min<size_t>(std::size(bldc), values.size());
          for (size_t i = 0; i < count; i++)
          {
              bldc[i] = static_cast<u_int8_t>(values[i] * conversion);
          }
          break;
        case SERVO: 
          count = std::min<size_t>(std::size(servo), values.size());
          for (size_t i = 0; i < count; i++)
          {
              servo[i] = static_cast<u_int16_t>(values[i] * conversion);
          }
          break;
        case STEPPER: 
          count = std::min<size_t>(std::size(stepper), values.size());
          for (size_t i = 0; i < count; i++)
          {
              stepper[i] = static_cast<u_int16_t>((values[i] + offsets[i]) * conversion);
          }
          break;
        default: return;
    }
  }

  void export_states(MotorType type, std::vector<double> &values, double conversion, std::vector<double> offsets = {}) 
  {
    size_t count = 0;

    switch (type)
    {
        case BLDC: 
          count = std::min<size_t>(std::size(bldc), values.size());
          for (size_t i = 0; i < count; i++)
          {
              values[i] = (bldc[i] * conversion);
          }
          break;
        case SERVO: 
          count = std::min<size_t>(std::size(servo), values.size());
          for (size_t i = 0; i < count; i++)
          {
              values[i] = (servo[i] * conversion);
          }
          break;
        case STEPPER: 
          count = std::min<size_t>(std::size(stepper), values.size());
          for (size_t i = 0; i < count; i++)
          {
              values[i] = (stepper[i] * conversion - offsets[i]);
          }
          break;
        default: return;
    }
  }
};

#endif // RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP