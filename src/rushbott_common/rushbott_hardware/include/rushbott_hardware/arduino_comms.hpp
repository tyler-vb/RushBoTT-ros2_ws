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

  void connect(const std::string &serial_device, int32_t baud_rate, int8_t msg_attempts, int16_t timeout_ms)
  {  
    msg_attempts_ = msg_attempts;
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    
    for (int attempt = 0; attempt < msg_attempts; attempt++)  // Retry up to 10 times
    {
        // Create an empty message of the right size (filled with zeros)
        MotorPacket handshake_request;
        handshake_request.header = MotorPacket::HEY;

        MotorPacket handshake_response;

        if (send_packet(handshake_request, handshake_response))
        {
          std::cout << "Handshake successful! Arduino is ready." << std::endl;
          return;
        }
        else
        {
          continue;
        }
    }
    std::cerr << "[ERROR] Handshake failed! Could not establish connection with Arduino." << std::endl;
    serial_conn_.Close(); // Close connection if handshake fails
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  bool send_packet(MotorPacket const &msg_packet, MotorPacket &response_packet)
  {
    serial_conn_.FlushIOBuffers();

    msg_packet.calculate_checksum();

    // serialize packet
    LibSerial::DataBuffer msg(sizeof(MotorPacket));
    std::memcpy(msg.data(), &msg_packet, sizeof(MotorPacket));
    serial_conn_.Write(msg);

    LibSerial::DataBuffer response_buffer(sizeof(MotorPacket));
    try
    {
      serial_conn_.Read(response_buffer, msg.size(), timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
      std::cerr << "[ERROR] Msg timed out" << std::endl;
      return false;
    }

    MotorPacket temp_packet;
    std::memcpy(&temp_packet, response_buffer.data(), sizeof(MotorPacket));
    uint8_t recieved_checksum = temp_packet.calculate_checksum();

    if (recieved_checksum != temp_packet.checksum)
    {
        std::cerr << "[ERROR] Checksum mismatch!" << std::endl;
        return false;
    }

    // Check for NACK response
    if (temp_packet.header == MotorPacket::NACK)
    {
        std::cerr << "[ERROR] NACK received!" << std::endl;
        return false;
    }

    response_packet = temp_packet;
    return true;
  }

  void read_encoders(MotorPacket &encoder_packet)
  {
    MotorPacket request_packet;
    request_packet.header = MotorPacket::ENC;
    send_packet(request_packet, encoder_packet);
  }

  void set_motors(MotorPacket const &motor_packet)
  {
    MotorPacket response_packet;
    send_packet(motor_packet, response_packet);
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    int msg_attempts_;
};

#endif // RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP