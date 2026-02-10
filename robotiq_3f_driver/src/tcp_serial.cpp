#include <robotiq_3f_driver/tcp_serial.hpp>
#include <serial/serial.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <cstring>
#include <stdexcept>
#include <sstream>

namespace robotiq_3f_driver
{

TcpSerial::TcpSerial(const std::string& host, uint16_t port)
  : host_(host), tcp_port_(port), socket_fd_(-1), is_open_(false), timeout_(std::chrono::milliseconds(500))
{
}

TcpSerial::~TcpSerial()
{
  if (is_open_)
  {
    close();
  }
}

void TcpSerial::open()
{
  if (is_open_)
  {
    return;
  }

  if (host_.empty())
  {
    THROW(serial::IOException, "TCP host address not set");
  }

  connect_socket();
  is_open_ = true;
}

bool TcpSerial::is_open() const
{
  return is_open_;
}

void TcpSerial::close()
{
  if (!is_open_)
  {
    return;
  }

  disconnect_socket();
  is_open_ = false;
}

std::vector<uint8_t> TcpSerial::read(size_t size)
{
  if (!is_open_)
  {
    THROW(serial::IOException, "TCP socket is not open");
  }

  std::vector<uint8_t> data;
  data.reserve(size);

  size_t total_bytes_read = 0;
  auto start_time = std::chrono::steady_clock::now();

  while (total_bytes_read < size)
  {
    // Check timeout
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
    
    if (elapsed >= timeout_)
    {
      const auto error_msg = "Timeout: Requested " + std::to_string(size) + " bytes, but got " + 
                            std::to_string(total_bytes_read) + " bytes";
      THROW(serial::IOException, error_msg.c_str());
    }

    // Calculate remaining timeout
    auto remaining_timeout = timeout_ - elapsed;
    
    // Use poll to wait for data with timeout
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;
    
    int poll_result = poll(&pfd, 1, remaining_timeout.count());
    
    if (poll_result < 0)
    {
      THROW(serial::IOException, "Poll error on TCP socket");
    }
    else if (poll_result == 0)
    {
      // Timeout
      const auto error_msg = "Timeout: Requested " + std::to_string(size) + " bytes, but got " + 
                            std::to_string(total_bytes_read) + " bytes";
      THROW(serial::IOException, error_msg.c_str());
    }

    // Data is available, read it
    uint8_t buffer[256];
    size_t bytes_to_read = std::min(size - total_bytes_read, sizeof(buffer));
    
    ssize_t bytes_read = ::recv(socket_fd_, buffer, bytes_to_read, 0);
    
    if (bytes_read < 0)
    {
      THROW(serial::IOException, "Error reading from TCP socket");
    }
    else if (bytes_read == 0)
    {
      THROW(serial::IOException, "TCP connection closed by remote host");
    }

    data.insert(data.end(), buffer, buffer + bytes_read);
    total_bytes_read += bytes_read;
  }

  if (total_bytes_read != size)
  {
    const auto error_msg = "Requested " + std::to_string(size) + " bytes, but got " + std::to_string(total_bytes_read);
    THROW(serial::IOException, error_msg.c_str());
  }

  return data;
}

void TcpSerial::write(const std::vector<uint8_t>& data)
{
  if (!is_open_)
  {
    THROW(serial::IOException, "TCP socket is not open");
  }

  size_t total_bytes_written = 0;
  
  while (total_bytes_written < data.size())
  {
    ssize_t bytes_written = ::send(socket_fd_, data.data() + total_bytes_written, 
                                   data.size() - total_bytes_written, 0);
    
    if (bytes_written < 0)
    {
      THROW(serial::IOException, "Error writing to TCP socket");
    }
    
    total_bytes_written += bytes_written;
  }

  if (total_bytes_written != data.size())
  {
    const auto error_msg = "Attempted to write " + std::to_string(data.size()) + 
                          " bytes, but wrote " + std::to_string(total_bytes_written);
    THROW(serial::IOException, error_msg.c_str());
  }
}

void TcpSerial::set_port(const std::string& port)
{
  // For TCP, we interpret "port" as "host:port" format
  // e.g., "192.168.1.11:502"
  size_t colon_pos = port.find(':');
  
  if (colon_pos != std::string::npos)
  {
    host_ = port.substr(0, colon_pos);
    tcp_port_ = static_cast<uint16_t>(std::stoul(port.substr(colon_pos + 1)));
  }
  else
  {
    // Assume it's just a host, use default Modbus TCP port
    host_ = port;
    tcp_port_ = 502;
  }
}

std::string TcpSerial::get_port() const
{
  return host_ + ":" + std::to_string(tcp_port_);
}

void TcpSerial::set_timeout(std::chrono::milliseconds timeout_ms)
{
  timeout_ = timeout_ms;
}

std::chrono::milliseconds TcpSerial::get_timeout() const
{
  return timeout_;
}

void TcpSerial::set_baudrate(uint32_t baudrate)
{
  // Baudrate is not applicable for TCP, but we implement it for interface compatibility
  // This is a no-op for TCP connections
  (void)baudrate;
}

uint32_t TcpSerial::get_baudrate() const
{
  // Return a dummy value for interface compatibility
  return 0;
}

void TcpSerial::set_host(const std::string& host)
{
  host_ = host;
}

std::string TcpSerial::get_host() const
{
  return host_;
}

void TcpSerial::set_tcp_port(uint16_t port)
{
  tcp_port_ = port;
}

uint16_t TcpSerial::get_tcp_port() const
{
  return tcp_port_;
}

void TcpSerial::connect_socket()
{
  // Create socket
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0)
  {
    THROW(serial::IOException, "Failed to create TCP socket");
  }

  // Resolve hostname
  struct addrinfo hints, *result, *rp;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;     // Allow IPv4 or IPv6
  hints.ai_socktype = SOCK_STREAM; // TCP socket
  hints.ai_flags = 0;
  hints.ai_protocol = 0;

  std::string port_str = std::to_string(tcp_port_);
  int status = getaddrinfo(host_.c_str(), port_str.c_str(), &hints, &result);
  
  if (status != 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
    std::string error_msg = "Failed to resolve hostname: ";
    error_msg += gai_strerror(status);
    THROW(serial::IOException, error_msg.c_str());
  }

  // Try each address until we successfully connect
  for (rp = result; rp != nullptr; rp = rp->ai_next)
  {
    if (::connect(socket_fd_, rp->ai_addr, rp->ai_addrlen) == 0)
    {
      break; // Success
    }
  }

  freeaddrinfo(result);

  if (rp == nullptr)
  {
    // No address succeeded
    ::close(socket_fd_);
    socket_fd_ = -1;
    std::string error_msg = "Failed to connect to " + host_ + ":" + std::to_string(tcp_port_);
    THROW(serial::IOException, error_msg.c_str());
  }

  // Set socket to non-blocking mode for timeout support
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
    THROW(serial::IOException, "Failed to get socket flags");
  }

  if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
    THROW(serial::IOException, "Failed to set socket to non-blocking mode");
  }

  // Set TCP_NODELAY to disable Nagle's algorithm for lower latency
  int flag = 1;
  if (setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int)) < 0)
  {
    // Non-fatal, just log a warning would be good but we don't have logger here
    // Continue without TCP_NODELAY
  }
}

void TcpSerial::disconnect_socket()
{
  if (socket_fd_ >= 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

}  // namespace robotiq_3f_driver

