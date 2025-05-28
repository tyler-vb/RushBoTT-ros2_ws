#ifndef RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP
#define RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP

#include <libserial/SerialPort.h>
#include <sstream>
#include <iostream>
#include <cstring>
#include <vector>

#include "rushbott_hardware/motor_packet.hpp"
#include "joint_group.hpp"


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
      std::cerr << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
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

    if (send_packet(config_packet, 10, true))
    {
      std::cout << get_timestamp() << "Connected to Arduino" << std::endl;
      return true;
    }

    std::cerr << get_timestamp() << "[ERROR] Could not establish connection with Arduino." << std::endl;
    serial_conn_.Close(); 
    return false;
  }

  bool send_packet(MotorPacket &packet, int timeout_ms, bool print_error= false, bool print_packets = false)
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

    std::stringstream ss;

    // ss << get_timestamp() << "Sending Bytes: " << "[ ";
    // for (uint8_t byte : buffer)
    // {
    //   ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << std::dec << " ";
    // }
    // ss << "]" << std::endl;

    serial_conn_.Write(buffer);

    // ss << get_timestamp() << "Bytes Sent" << std::endl;

    size_t byte_index = 0;

    buffer.clear();
    buffer.resize(sizeof(MotorPacket));

    auto start_time = std::chrono::steady_clock::now();

    uint8_t byte = 0x00;

    while (byte_index < buffer.size())
    {
      if (serial_conn_.IsDataAvailable())
      {
        serial_conn_.ReadByte(byte, 0.01);

        if (byte_index > 1 || byte_index == 0 && byte == 0x64 || byte_index == 1 && byte == packet.id)
        {
          // ss << get_timestamp() << "Recieved Byte " << byte_index+1;
          // ss << " [" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << std::dec << "]" << std::endl;
          buffer[byte_index] = byte;
          byte_index++;
        }
        else
        {
          // ss << get_timestamp() << "Recieved Invalid Byte " << byte_index+1;
          // ss << " [" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << std::dec << "]" << std::endl;
          byte_index = 0;
          buffer[0] = 0x00;
          buffer[1] = 0x00;
        }
      }

      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time
      ).count();

      if (timeout_ms > 0 && elapsed_time >= timeout_ms)
      {
        break;
      }
    }

    // std::cout << ss.str() << get_timestamp() << "Packet handeling finished" << std::endl;

    std::memcpy(&incoming_packet_, buffer.data(), sizeof(MotorPacket));
    
    // uint32_t seconds = packet.time / 1'000'000;
    // uint32_t micros  = packet.time % 1'000'000;
    // std::cout << get_timestamp() << "Packet " << packet.id << "took " << seconds << '.' << std::setw(6) << std::setfill('0') << micros << std::endl;

    if (print_packets)
    {
      incoming_packet_.print_packet("Recieving packet: ");
    }

    if (byte_index < buffer.size())
    {
      error = "[ERROR] Message timed out";
    }

    else if (incoming_packet_.calculate_checksum() != incoming_packet_.checksum)
    {
      error = "[ERROR] Checksum mismatch!";
    }

    else if (incoming_packet_.flag == MotorPacket::NACK)
    {
      error = "[ERROR] NACK received!";
    }
    else
    {
      packet = incoming_packet_;
      packet.id++;
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

    packet.id++;
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
    return send_packet(encoder_packet, timeout_ms_, true);
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

    if (!send_packet(motor_packet, timeout_ms_, true))
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
    // Get current time since epoch in microseconds
    auto now = std::chrono::system_clock::now();
    auto us_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(
                            now.time_since_epoch()
                          ).count();

    // Split into seconds and microsecond remainder
    uint64_t seconds = us_since_epoch / 1'000'000;
    uint64_t micros  = us_since_epoch % 1'000'000;

    // Format as "seconds.microseconds"
    std::ostringstream oss;
    oss << "[" << seconds
        << '.'
        << std::setw(6) << std::setfill('0')
        << micros << "] ";
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
    MotorPacket incoming_packet_ = {};
};

#endif // RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP