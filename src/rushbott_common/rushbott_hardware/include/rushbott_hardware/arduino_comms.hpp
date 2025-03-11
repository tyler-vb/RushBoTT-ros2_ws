#ifndef RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP
#define RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP

#include <libserial/SerialPort.h>
#include <sstream>
#include <iostream>
#include <cstring>
#include <vector>

#include "rushbott_hardware/motor_packet.hpp"


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  bool connect(const std::string &serial_device, int32_t baud_rate, int8_t msg_attempts, int16_t timeout_ms)
  {  
    msg_attempts_ = msg_attempts;
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));

    std::cout << "Trying to connect to Arduino..." << std::endl;
    
    for (int attempt = 0; attempt < msg_attempts; attempt++) 
    {
        // Create an empty message of the right size (filled with zeros)
        MotorPacket handshake_packet = {};
        handshake_packet.flag = MotorPacket::HEY;

        if (send_packet(handshake_packet, 2000))
        {
          std::cout << "Handshake successful! Arduino is ready." << std::endl;
          return true;
        }
        else
        {
          continue;
        }
    }
    std::cerr << "[ERROR] Handshake failed! Could not establish connection with Arduino." << std::endl;
    serial_conn_.Close(); // Close connection if handshake fails
    return false;
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  bool send_packet(MotorPacket &packet, int timeout = 0)
  {
    if (timeout == 0)
    {
      timeout = timeout_ms_;
    }

    serial_conn_.FlushIOBuffers();

    packet.checksum = packet.calculate_checksum();

    // serialize packet
    LibSerial::DataBuffer buffer(sizeof(MotorPacket));
    std::memcpy(buffer.data(), &packet, sizeof(MotorPacket));
    serial_conn_.Write(buffer);

    try
    {
      serial_conn_.Read(buffer, buffer.size(), timeout);
    }
    catch (const LibSerial::ReadTimeout&)
    {
      std::cerr << "[ERROR] Msg timed out" << std::endl;
      return false;
    }

    std::memcpy(&packet, buffer.data(), sizeof(MotorPacket));

    if (packet.calculate_checksum() != packet.checksum)
    {
        std::cerr << "[ERROR] Checksum mismatch!" << std::endl;
        return false;
    }

    // Check for NACK response
    if (packet.flag == MotorPacket::NACK)
    {
        std::cerr << "[ERROR] NACK received!" << std::endl;
        return false;
    }

    return true;
  }

  bool read_encoders(MotorPacket &encoder_packet)
  {
    encoder_packet.flag = MotorPacket::ENC;
    return send_packet(encoder_packet);
  }

  bool set_motors(MotorPacket &motor_packet)
  {
    motor_packet.flag = MotorPacket::MOT;
    return send_packet(motor_packet);
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    int msg_attempts_;
};

#endif // RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP