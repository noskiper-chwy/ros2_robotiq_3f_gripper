#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <robotiq_3f_driver/serial.hpp>

namespace robotiq_3f_driver
{
/**
 * TCP/IP implementation of the Serial interface for ethernet communication
 * with the Robotiq 3F gripper using Modbus TCP protocol.
 */
class TcpSerial : public Serial
{
public:
  /**
   * Creates a TCP Serial object to send and receive bytes over TCP/IP.
   * @param host The IP address or hostname of the gripper (e.g., "192.168.1.11")
   * @param port The TCP port number (default is 502 for Modbus TCP)
   */
  TcpSerial(const std::string& host = "", uint16_t port = 502);
  
  ~TcpSerial() override;

  void open() override;

  [[nodiscard]] bool is_open() const override;

  void close() override;

  [[nodiscard]] std::vector<uint8_t> read(size_t size = 1) override;
  void write(const std::vector<uint8_t>& data) override;

  void set_port(const std::string& port) override;
  [[nodiscard]] std::string get_port() const override;

  void set_timeout(std::chrono::milliseconds timeout_ms) override;
  [[nodiscard]] std::chrono::milliseconds get_timeout() const override;

  void set_baudrate(uint32_t baudrate) override;
  [[nodiscard]] uint32_t get_baudrate() const override;

  /**
   * Set the TCP host address
   * @param host IP address or hostname (e.g., "192.168.1.11")
   */
  void set_host(const std::string& host);
  
  /**
   * Get the TCP host address
   * @return The current host address
   */
  [[nodiscard]] std::string get_host() const;

  /**
   * Set the TCP port number
   * @param port Port number (typically 502 for Modbus TCP)
   */
  void set_tcp_port(uint16_t port);
  
  /**
   * Get the TCP port number
   * @return The current TCP port number
   */
  [[nodiscard]] uint16_t get_tcp_port() const;

private:
  std::string host_;
  uint16_t tcp_port_;
  int socket_fd_;
  bool is_open_;
  std::chrono::milliseconds timeout_;
  
  // Helper methods
  void connect_socket();
  void disconnect_socket();
};
}  // namespace robotiq_3f_driver

