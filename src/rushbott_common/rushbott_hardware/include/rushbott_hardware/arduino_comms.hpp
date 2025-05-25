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

  bool connect(MotorPacket &config_packet, const std::string &serial_device, int32_t baud_rate, int16_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));

    std::this_thread::sleep_for(std::chrono::seconds(2));

    config_packet.flag = MotorPacket::HEY;

    if (send_packet(config_packet, false))
    {
      std::cout << get_timestamp() << "Connected to Arduino" << std::endl;
      return true;
    }

    std::cerr << get_timestamp() << "[ERROR] Could not establish connection with Arduino." << std::endl;
    serial_conn_.Close(); 
    return false;
  }

  bool send_packet(MotorPacket &packet, bool print_error = false, bool print_packets = false)
  {
    std::string error = "";

    packet.checksum = packet.calculate_checksum();

    if (print_packets)
    {
      packet.print_packet("Sending packet: ");
    }
    

    // serialize packet
    LibSerial::DataBuffer buffer(sizeof(MotorPacket));
    std::memcpy(buffer.data(), &packet, sizeof(MotorPacket));

    serial_conn_.Write(buffer);

    size_t byte_count = 0;

    buffer.clear();
    buffer.resize(sizeof(MotorPacket));

    auto start_time = std::chrono::steady_clock::now();

    while (byte_count < buffer.size())
    {
      if (serial_conn_.IsDataAvailable())
      {
        serial_conn_.ReadByte(buffer[byte_count], 1);

        if (byte_count > 0 || buffer[0] == 0x64)
        {
            byte_count++;
        }
      }

      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time
      ).count();

      if (timeout_ms_ > 0 && elapsed_time >= timeout_ms_)
      {
        break;
      }
    }

    MotorPacket received_packet = {};
    std::memcpy(&received_packet, buffer.data(), sizeof(MotorPacket));

    if (print_packets)
    {
      received_packet.print_packet("Recieving packet: ");
    }

    if (byte_count < buffer.size())
    {
      error = "[ERROR] Message timed out";
    }

    else if (received_packet.calculate_checksum() != received_packet.checksum)
    {
      error = "[ERROR] Checksum mismatch!";
    }

    else if (received_packet.flag == MotorPacket::NACK)
    {
      error = "[ERROR] NACK received!";
    }
    else
    {
      packet = received_packet;
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

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  bool read_encoders(MotorPacket &encoder_packet)
  {
    encoder_packet.flag = MotorPacket::ENC;
    return send_packet(encoder_packet, true);
  }

  bool set_motors(MotorPacket &motor_packet, bool &is_calibrating)
  {
    if (is_calibrating)
    {
      motor_packet.flag = MotorPacket::CAL;
    }
    else
    {
      motor_packet.flag = MotorPacket::MOT;
    }

    if (!send_packet(motor_packet, true, true))
    {
      return false;
    }
    else if (motor_packet.flag == MotorPacket::CAL)
    {
      is_calibrating = false;
    }
    return true;
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

    // bool send_with_retries(MotorPacket &packet, int timeout_ms)
  // {
  //   uint8_t initial_flag = packet.flag;
  //   auto start_time = std::chrono::steady_clock::now();

  //   while (true)
  //   {
  //     if (send_packet(packet, true))
  //     {
  //       if (packet.flag == MotorPacket::ACK)
  //       {
  //         packet.flag = initial_flag;
  //       }
  //       else
  //       {
  //         return true;
  //       }

  //       auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
  //         std::chrono::steady_clock::now() - start_time
  //       ).count();
    
  //       if (timeout_ms > 0 && elapsed_time >= timeout_ms)
  //       {
  //         break;
  //       }
  //     }
  //   }
  //   return false; 
  // }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    int msg_attempts_;
};

#endif // RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP