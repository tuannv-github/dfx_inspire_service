#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <algorithm>
#include <termios.h>
#include <sys/select.h>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <queue>
#include <vector>

class SerialPort
{
public:
  using SharedPtr = std::shared_ptr<SerialPort>;

  SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2)
  {
    set_timeout(timeout_ms);
    Init(port, baudrate);
  }

  ~SerialPort()
  {
    close(fd_);
  }

  ssize_t send(const uint8_t* data, size_t len)
  {
    ssize_t ret = ::write(fd_, data, len);
    return ret;
  }

  ssize_t recv(uint8_t* data, size_t len)
  {
    if (rx_.empty())
      pull(timeout_.tv_sec * 1000 + timeout_.tv_usec / 1000);
    size_t n = std::min(len, rx_.size());
    if (n == 0)
      return 0;
    std::memcpy(data, rx_.data(), n);
    rx_.erase(rx_.begin(), rx_.begin() + n);
    return static_cast<ssize_t>(n);
  }

  // Drop unread bytes so the next reply cannot continue a misaligned frame.
  void flush_input()
  {
    rx_.clear();
    tcflush(fd_, TCIFLUSH);
  }

  // Read one frame. The hand replies with header 90 EB; commands use EB 90.
  // length is the byte after the id, and the frame is 5 + length bytes.
  bool read_frame(uint8_t* data, size_t cap, size_t& out_len, int timeout_ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true)
    {
      if (take_frame(data, cap, out_len))
        return true;
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline)
        return false;
      int remain = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
      if (remain < 1)
        remain = 1;
      pull(remain);
    }
  }

  void set_timeout(int timeout_ms)
  {
    timeout_.tv_sec = timeout_ms / 1000;
    timeout_.tv_usec = (timeout_ms % 1000) * 1000;
  }

private:
  void Init(std::string port, speed_t baudrate)
  {
    int ret;
    // Open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
      printf("Open serial port %s failed\n", port.c_str());
      exit(-1);
    }

    // Set attributes
    struct termios option;
    memset(&option, 0, sizeof(option));
    ret = tcgetattr(fd_, &option);

    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_iflag = 0;

    cfsetispeed(&option, baudrate);
    cfsetospeed(&option, baudrate);

    option.c_cflag &= ~CSIZE;
    option.c_cflag |= CS8; // 8
    option.c_cflag &= ~PARENB; // no parity
    option.c_iflag &= ~INPCK; // no parity
    option.c_cflag &= ~CSTOPB; // 1 stop bit

    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = 0;
    option.c_lflag |= CBAUDEX;

    ret = tcflush(fd_, TCIFLUSH);
    ret = tcsetattr(fd_, TCSANOW, &option);
  }

  bool pull(int timeout_ms)
  {
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(fd_, &rset);
    // select() overwrites the timeval on Linux. Keep the caller's timeout.
    timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    if (select(fd_ + 1, &rset, NULL, NULL, &tv) <= 0)
      return false;
    uint8_t tmp[256];
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n <= 0)
      return false;
    rx_.insert(rx_.end(), tmp, tmp + n);
    if (rx_.size() > 1024)
      rx_.erase(rx_.begin(), rx_.end() - 512);
    return true;
  }

  bool take_frame(uint8_t* data, size_t cap, size_t& out_len)
  {
    while (rx_.size() >= 2)
    {
      size_t start = 0;
      while (start + 1 < rx_.size() && !frame_header(start))
        ++start;
      if (start > 0)
        rx_.erase(rx_.begin(), rx_.begin() + start);
      if (rx_.size() < 4)
        return false;
      const uint8_t length = rx_[3];
      if (length > 64)
      {
        rx_.erase(rx_.begin());
        continue;
      }
      const size_t total = static_cast<size_t>(length) + 5;
      if (rx_.size() < total)
        return false;
      uint8_t sum = 0;
      for (size_t i = 2; i + 1 < total; ++i)
        sum = static_cast<uint8_t>(sum + rx_[i]);
      if (sum != rx_[total - 1] || total > cap)
      {
        rx_.erase(rx_.begin());
        continue;
      }
      std::memcpy(data, rx_.data(), total);
      rx_.erase(rx_.begin(), rx_.begin() + total);
      out_len = total;
      return true;
    }
    if (rx_.size() == 1 && rx_[0] != 0xEB && rx_[0] != 0x90)
      rx_.clear();
    return false;
  }

  bool frame_header(size_t index) const
  {
    const uint8_t a = rx_[index];
    const uint8_t b = rx_[index + 1];
    return (a == 0xEB && b == 0x90) || (a == 0x90 && b == 0xEB);
  }

  int fd_;
  fd_set rSet_;
  timeval timeout_;
  std::vector<uint8_t> rx_;
};

#endif // SERIAL_PORT_H