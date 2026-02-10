#pragma once

#include <robotiq_3f_driver/tcp_serial.hpp>

namespace robotiq_3f_driver
{
/**
 * Modbus TCP wrapper that converts between Modbus RTU and Modbus TCP framing.
 * 
 * This class wraps TcpSerial and automatically:
 * - Converts Modbus RTU frames to Modbus TCP (adds MBAP header, removes CRC)
 * - Converts Modbus TCP responses back to RTU format (removes MBAP header, adds CRC)
 * 
 * This allows the existing Modbus RTU driver code to work over TCP/Ethernet.
 */
class ModbusTcpSerial : public Serial
{
public:
  /**
   * Creates a Modbus TCP Serial adapter.
   * @param host The IP address or hostname of the gripper
   * @param port The TCP port number (default is 502 for Modbus TCP)
   */
  ModbusTcpSerial(const std::string& host = "", uint16_t port = 502);
  
  ~ModbusTcpSerial() override = default;

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
   * Set the Modbus unit ID (slave address) for MBAP header
   * @param unit_id The unit ID (typically same as slave address)
   */
  void set_unit_id(uint8_t unit_id);

private:
  std::unique_ptr<TcpSerial> tcp_serial_;
  uint16_t transaction_id_;
  uint8_t unit_id_;
  
  /**
   * Convert Modbus RTU frame to Modbus TCP frame
   * RTU: [Slave][Function][Data...][CRC-16]
   * TCP: [MBAP Header][Function][Data...]
   */
  std::vector<uint8_t> rtu_to_tcp(const std::vector<uint8_t>& rtu_frame);
  
  /**
   * Convert Modbus TCP frame to Modbus RTU frame
   * TCP: [MBAP Header][Function][Data...]
   * RTU: [Slave][Function][Data...][CRC-16]
   */
  std::vector<uint8_t> tcp_to_rtu(const std::vector<uint8_t>& tcp_frame);
  
  /**
   * Calculate Modbus RTU CRC-16
   */
  uint16_t calculate_crc16(const std::vector<uint8_t>& data);
};
}  // namespace robotiq_3f_driver

