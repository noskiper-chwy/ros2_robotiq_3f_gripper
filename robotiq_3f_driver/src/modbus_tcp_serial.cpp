#include <robotiq_3f_driver/modbus_tcp_serial.hpp>
#include <serial/serial.h>

namespace robotiq_3f_driver
{

ModbusTcpSerial::ModbusTcpSerial(const std::string& host, uint16_t port)
  : tcp_serial_(std::make_unique<TcpSerial>(host, port)), transaction_id_(0), unit_id_(0x09)
{
}

void ModbusTcpSerial::open()
{
  tcp_serial_->open();
}

bool ModbusTcpSerial::is_open() const
{
  return tcp_serial_->is_open();
}

void ModbusTcpSerial::close()
{
  tcp_serial_->close();
}

std::vector<uint8_t> ModbusTcpSerial::read(size_t size)
{
  // For Modbus TCP, we need to:
  // 1. Read the MBAP header (7 bytes) first
  // 2. Extract the length field to know how much more to read  
  // 3. Read the remaining data
  // 4. Convert back to RTU format by removing MBAP header and adding CRC
  
  // Note: The 'size' parameter is the expected RTU frame size, but we determine
  // the actual read size from the Modbus TCP MBAP header
  (void)size;  // Unused for TCP - we get size from MBAP header
  
  try
  {
    // Read MBAP header (7 bytes)
    // Format: Transaction ID (2) | Protocol ID (2) | Length (2) | Unit ID (1)
    std::vector<uint8_t> mbap_header = tcp_serial_->read(7);
    
    if (mbap_header.size() != 7)
    {
      THROW(serial::IOException, "Failed to read complete MBAP header");
    }
    
    // Extract length from MBAP header (bytes 4-5, big-endian)
    uint16_t length = (static_cast<uint16_t>(mbap_header[4]) << 8) | mbap_header[5];
    
    // Length field = Unit ID (1 byte already in header[6]) + PDU (function code + data)
    // So we need to read (length - 1) more bytes for the PDU
    if (length < 1)
    {
      THROW(serial::IOException, "Invalid Modbus TCP length field");
    }
    
    size_t remaining_bytes = length - 1;
    std::vector<uint8_t> pdu = tcp_serial_->read(remaining_bytes);
    
    if (pdu.size() != remaining_bytes)
    {
      const auto error_msg = "Expected " + std::to_string(remaining_bytes) + 
                            " bytes for PDU, but got " + std::to_string(pdu.size());
      THROW(serial::IOException, error_msg.c_str());
    }
    
        // Combine: MBAP header + PDU
        std::vector<uint8_t> tcp_frame;
        tcp_frame.reserve(mbap_header.size() + pdu.size());
        tcp_frame.insert(tcp_frame.end(), mbap_header.begin(), mbap_header.end());
        tcp_frame.insert(tcp_frame.end(), pdu.begin(), pdu.end());

        // Convert TCP frame to RTU frame (adds slave address and CRC)
        std::vector<uint8_t> rtu_frame = tcp_to_rtu(tcp_frame);

        return rtu_frame;
  }
  catch (const serial::IOException& e)
  {
    // Re-throw with context
    std::string error_msg = "Modbus TCP read failed: ";
    error_msg += e.what();
    THROW(serial::IOException, error_msg.c_str());
  }
}

void ModbusTcpSerial::write(const std::vector<uint8_t>& data)
{
  // Convert Modbus RTU frame to Modbus TCP frame
  std::vector<uint8_t> tcp_frame = rtu_to_tcp(data);

  // Send over TCP
  tcp_serial_->write(tcp_frame);
}

void ModbusTcpSerial::set_port(const std::string& port)
{
  tcp_serial_->set_port(port);
}

std::string ModbusTcpSerial::get_port() const
{
  return tcp_serial_->get_port();
}

void ModbusTcpSerial::set_timeout(std::chrono::milliseconds timeout_ms)
{
  tcp_serial_->set_timeout(timeout_ms);
}

std::chrono::milliseconds ModbusTcpSerial::get_timeout() const
{
  return tcp_serial_->get_timeout();
}

void ModbusTcpSerial::set_baudrate(uint32_t baudrate)
{
  // No-op for TCP
  (void)baudrate;
}

uint32_t ModbusTcpSerial::get_baudrate() const
{
  return 0;
}

void ModbusTcpSerial::set_unit_id(uint8_t unit_id)
{
  unit_id_ = unit_id;
}

std::vector<uint8_t> ModbusTcpSerial::rtu_to_tcp(const std::vector<uint8_t>& rtu_frame)
{
  // Modbus RTU frame: [Slave Address][Function Code][Data...][CRC-16 (2 bytes)]
  // Modbus TCP frame: [Transaction ID (2)][Protocol ID (2)][Length (2)][Unit ID (1)][Function Code][Data...]
  
  // Per Robotiq 3F manual section 4.8.1:
  // - RTU uses registers 0x03E8 (write) and 0x07D0 (read)
  // - TCP uses registers 0x0000 (both write and read)
  // - RTU uses FC03 (Read Holding Registers)
  // - TCP uses FC04 (Read Input Registers)
  
  if (rtu_frame.size() < 4)  // Minimum: slave + function + CRC(2)
  {
    THROW(serial::IOException, "Invalid Modbus RTU frame: too short");
  }
  
  // Extract components from RTU frame
  uint8_t slave_address = rtu_frame[0];
  uint8_t function_code = rtu_frame[1];
  
  // Remove slave address and CRC (last 2 bytes) from RTU frame
  std::vector<uint8_t> pdu(rtu_frame.begin() + 1, rtu_frame.end() - 2);  // Function code + data
  
  // Translate RTU to TCP for Robotiq 3F gripper
  // 1. Convert FC03 (Read Holding Registers) to FC04 (Read Input Registers)
  if (function_code == 0x03) {
    pdu[0] = 0x04;  // Change function code to FC04
  }
  
  // 2. Translate register addresses from RTU space to TCP space
  if (pdu.size() >= 3) {
    uint16_t address = (static_cast<uint16_t>(pdu[1]) << 8) | pdu[2];
    
    // RTU addresses: 0x03E8 (1000) for writes, 0x07D0 (2000) for reads
    // TCP addresses: 0x0000 (0) for both
    if (address == 0x03E8 || address == 0x07D0) {
      pdu[1] = 0x00;  // Address high byte
      pdu[2] = 0x00;  // Address low byte
    }
  }
  
  // Build Modbus TCP MBAP header
  std::vector<uint8_t> tcp_frame;
  tcp_frame.reserve(7 + pdu.size());
  
  // Transaction ID (2 bytes, big-endian)
  transaction_id_++;
  tcp_frame.push_back((transaction_id_ >> 8) & 0xFF);
  tcp_frame.push_back(transaction_id_ & 0xFF);
  
  // Protocol ID (2 bytes, always 0x0000 for Modbus)
  tcp_frame.push_back(0x00);
  tcp_frame.push_back(0x00);
  
  // Length (2 bytes, big-endian): unit ID (1) + PDU length
  uint16_t length = 1 + pdu.size();
  tcp_frame.push_back((length >> 8) & 0xFF);
  tcp_frame.push_back(length & 0xFF);
  
  // Unit ID (1 byte) - use configured unit_id_ (not slave_address from RTU)
  tcp_frame.push_back(unit_id_);
  
  // Append PDU (function code + data)
  tcp_frame.insert(tcp_frame.end(), pdu.begin(), pdu.end());
  
  return tcp_frame;
}

std::vector<uint8_t> ModbusTcpSerial::tcp_to_rtu(const std::vector<uint8_t>& tcp_frame)
{
  // Modbus TCP frame: [Transaction ID (2)][Protocol ID (2)][Length (2)][Unit ID (1)][Function Code][Data...]
  // Modbus RTU frame: [Slave Address][Function Code][Data...][CRC-16 (2 bytes)]
  
  if (tcp_frame.size() < 8)  // Minimum: MBAP header (7) + function code (1)
  {
    THROW(serial::IOException, "Invalid Modbus TCP frame: too short");
  }
  
  // Extract Unit ID from MBAP header (byte 6)
  uint8_t unit_id = tcp_frame[6];
  
  // Extract PDU (function code + data) - everything after MBAP header
  std::vector<uint8_t> pdu(tcp_frame.begin() + 7, tcp_frame.end());
  
  // Translate TCP response back to RTU format for Robotiq 3F gripper
  // Convert FC04 (Read Input Registers) back to FC03 (Read Holding Registers)
  if (pdu.size() > 0 && pdu[0] == 0x04) {
    pdu[0] = 0x03;  // Change function code back to FC03
  }
  // Also handle exception responses (0x84 is FC04 exception, convert to 0x83 which is FC03 exception)
  if (pdu.size() > 0 && pdu[0] == 0x84) {
    pdu[0] = 0x83;  // Change exception code back to FC03 exception
  }
  
  // Build RTU frame: slave address + PDU
  std::vector<uint8_t> rtu_frame;
  rtu_frame.reserve(1 + pdu.size() + 2);
  
  // Add slave address (unit ID)
  rtu_frame.push_back(unit_id);
  
  // Add PDU
  rtu_frame.insert(rtu_frame.end(), pdu.begin(), pdu.end());
  
  // Calculate and append CRC-16
  uint16_t crc = calculate_crc16(rtu_frame);
  rtu_frame.push_back(crc & 0xFF);        // CRC low byte
  rtu_frame.push_back((crc >> 8) & 0xFF); // CRC high byte
  
  return rtu_frame;
}

uint16_t ModbusTcpSerial::calculate_crc16(const std::vector<uint8_t>& data)
{
  // Modbus CRC-16 (CRC-16-IBM/ANSI)
  uint16_t crc = 0xFFFF;
  
  for (uint8_t byte : data)
  {
    crc ^= byte;
    
    for (int i = 0; i < 8; ++i)
    {
      if (crc & 0x0001)
      {
        crc = (crc >> 1) ^ 0xA001;
      }
      else
      {
        crc = crc >> 1;
      }
    }
  }
  
  return crc;
}

}  // namespace robotiq_3f_driver

