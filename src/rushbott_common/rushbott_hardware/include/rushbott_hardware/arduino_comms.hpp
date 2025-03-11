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

    std::cout << get_timestamp() << "Trying to connect to Arduino..." << std::endl;
    
    for (int attempt = 0; attempt < msg_attempts; attempt++) 
    {
        // Create an empty message of the right size (filled with zeros)
        MotorPacket handshake_packet = {};
        handshake_packet.flag = MotorPacket::HEY;

        if (send_packet(handshake_packet, true, 2000))
        {
          std::cout << get_timestamp() << "Handshake successful! Arduino is ready." << std::endl;
          return true;
        }
        else
        {
          continue;
        }
    }
    std::cerr << get_timestamp() << "[ERROR] Handshake failed! Could not establish connection with Arduino." << std::endl;
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


  bool send_packet(MotorPacket &packet, bool print_error = false, int timeout = 0)
  {
    if (timeout == 0)
    {
      timeout = timeout_ms_;
    }

    std::string error = "";

    packet.checksum = packet.calculate_checksum();

    // serialize packet
    LibSerial::DataBuffer buffer(sizeof(MotorPacket));
    std::memcpy(buffer.data(), &packet, sizeof(MotorPacket));

    serial_conn_.Write(buffer);

    int byte_count = 0;

    buffer.clear();
    buffer.resize(sizeof(MotorPacket));

    auto start_time = std::chrono::steady_clock::now();

    while (byte_count < buffer.size())
    {
      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time
      ).count();

      if (elapsed_time >= timeout)
      {
        break;
      }

      try
      {
        serial_conn_.ReadByte(buffer[byte_count], 10);
      }
      catch (const LibSerial::ReadTimeout &)
      {
        continue;
      }

      if (byte_count > 0 || buffer[0] == 0x64)
      {
          byte_count++;
      }
    }

    std::memcpy(&packet, buffer.data(), sizeof(MotorPacket));

    if (byte_count < buffer.size())
    {
      error = "[ERROR] Message timed out";
    }

    else if (packet.calculate_checksum() != packet.checksum)
    {
      error = "[ERROR] Checksum mismatch!";
    }

    else if (packet.flag == MotorPacket::NACK)
    {
      error = "[ERROR] NACK received!";
    }
    else
    {
      return true;
    }

    if (print_error == true)
    {
      std::cerr << get_timestamp() << error << " [ ";
      for (uint8_t byte : buffer)
      {
        std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << std::dec << " ";
      }
      std::cerr << "]" << std::endl;
    }

    return false;
  }

  bool read_encoders(MotorPacket &encoder_packet)
  {
    encoder_packet.flag = MotorPacket::ENC;
    return send_packet(encoder_packet, true);
  }

  bool set_motors(MotorPacket &motor_packet)
  {
    motor_packet.flag = MotorPacket::MOT;
    return send_packet(motor_packet, true);
  }

  std::string get_timestamp()
  {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&time_t_now);

    std::ostringstream oss;
    oss << "[" << std::put_time(&bt, "%H:%M:%S") << "." << std::setw(3) << std::setfill('0') << ms.count() << "] ";
    return oss.str();
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    int msg_attempts_;
};

#endif // RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP