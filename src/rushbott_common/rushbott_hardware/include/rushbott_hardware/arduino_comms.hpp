#ifndef RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP
#define RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


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

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));

    std::string response;
    
    for (int attempt = 0; attempt < 5; attempt++)  // Retry up to 5 times
    {
        serial_conn_.FlushIOBuffers();
        serial_conn_.Write("h");  // Send handshake request
        std::cout << "Waiting for Arduino to respond..." << std::endl;

        try
        {
          serial_conn_.ReadLine(response, '\n', timeout_ms);
          response.erase(std::remove(response.begin(), response.end(), '\r'), response.end());
          response.erase(std::remove(response.begin(), response.end(), '\n'), response.end());

          if (response == "h")
          {
            std::cout << "Handshake successful! Arduino is ready." << std::endl;
              return;
          }
          else {
            std::cerr << "[WARNING] Invalid response: " << response << std::endl;
          }
        } 
        catch (const LibSerial::ReadTimeout&)
        {
          std::cerr << "[WARNING] Handshake attempt timed out. Retrying..." << std::endl;
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


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";

    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
      std::cerr << "[WARNING] The ReadLine() call has timed out" << std::endl;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("");
  }

  std::vector<int> read_encoder_values()
  {
    std::string response = send_msg("e", true);

    std::vector<int> values;
    std::stringstream ss(response);
    std::string token;

    // Split response by spaces and convert to integers
    while (std::getline(ss, token, ' '))
    {
      values.push_back(std::atoi(token.c_str()));  // Convert token to int and add to vector

    }

    return values;  // Return vector of encoder values
  }

  void set_motor_values(std::vector<double> cmd_values)
  {
    std::stringstream ss;
    ss << "m";

    for (const auto &val : cmd_values)
    {
        ss << " " << val;
    }

    // send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // RUSHBOTT_HARDWARE_ARDUINO_COMMS_HPP