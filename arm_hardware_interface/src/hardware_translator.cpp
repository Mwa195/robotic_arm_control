#include "arm_hardware_interface/hardware_translator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// POSIX serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

// TCP socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

// String parsing
#include <sstream>
#include <stdexcept>

namespace arm_hardware_interface
{

// ─────────────────────────────────────────────────────────────────────────────
// on_init
// Reads "transport" param ("serial" or "wifi") then reads the matching
// connection params. Everything else (joint validation, array init) is identical
// to the serial-only version.
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // ── Read transport selection ──────────────────────────────────────────────
  try {
    transport_ = info_.hardware_parameters.at("transport");
  } catch (const std::out_of_range &) {
    // Default to serial if param is absent — backward compatible
    transport_ = "serial";
    RCLCPP_WARN(logger_, "No 'transport' param found — defaulting to 'serial'.");
  }

  if (transport_ != "serial" && transport_ != "wifi") {
    RCLCPP_FATAL(logger_,
      "Unknown transport '%s'. Must be 'serial' or 'wifi'.", transport_.c_str());
    return CallbackReturn::ERROR;
  }

  // ── Read transport-specific params ────────────────────────────────────────
  try {
    if (transport_ == "serial") {
      port_      = info_.hardware_parameters.at("serial_port");
      baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
      RCLCPP_INFO(logger_, "Transport: SERIAL — port: %s @ %d baud",
        port_.c_str(), baud_rate_);
    } else {
      esp_ip_   = info_.hardware_parameters.at("esp_ip");
      esp_port_ = std::stoi(info_.hardware_parameters.at("esp_port"));
      RCLCPP_INFO(logger_, "Transport: WIFI — %s:%d", esp_ip_.c_str(), esp_port_);
    }
  } catch (const std::out_of_range &) {
    if (transport_ == "serial") {
      RCLCPP_FATAL(logger_,
        "URDF missing serial params. Add:\n"
        "  <param name=\"serial_port\">/dev/ttyUSB0</param>\n"
        "  <param name=\"baud_rate\">115200</param>");
    } else {
      RCLCPP_FATAL(logger_,
        "URDF missing wifi params. Add:\n"
        "  <param name=\"esp_ip\">192.168.x.x</param>\n"
        "  <param name=\"esp_port\">8888</param>");
    }
    return CallbackReturn::ERROR;
  }

  serial_fd_ = -1;

  // ── Validate joints (unchanged) ───────────────────────────────────────────
  if (info_.joints.size() != 3) {
    RCLCPP_FATAL(logger_,
      "Expected 3 joints, got %zu. Check URDF <ros2_control> block.",
      info_.joints.size());
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(logger_,
        "Joint '%s' must have exactly one 'position' command interface.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    bool has_pos = false, has_vel = false;
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_pos = true;
      if (si.name == hardware_interface::HW_IF_VELOCITY)  has_vel = true;
    }
    if (!has_pos || !has_vel) {
      RCLCPP_FATAL(logger_,
        "Joint '%s' must have 'position' and 'velocity' state interfaces.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
  }

  position_state_.assign(3, 0.0);
  velocity_state_.assign(3, 0.0);
  position_command_.assign(3, 0.0);

  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure
// Branches on transport_ — opens serial port or TCP socket.
// From this point on, serial_fd_ is a valid POSIX fd regardless of transport.
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (transport_ == "serial") {
    if (!openSerialPort()) {
      RCLCPP_ERROR(logger_, "Failed to open serial port '%s': %s",
        port_.c_str(), strerror(errno));
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "Serial port %s opened successfully.", port_.c_str());
  } else {
    if (!openTcpSocket()) {
      RCLCPP_ERROR(logger_, "Failed to connect to ESP at %s:%d",
        esp_ip_.c_str(), esp_port_);
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "TCP connected to ESP at %s:%d", esp_ip_.c_str(), esp_port_);
  }

  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate / on_deactivate  (unchanged logic, works for both transports)
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  position_command_ = position_state_;
  RCLCPP_INFO(logger_, "Arm hardware interface activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(logger_, "Connection closed.");
  }
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces / export_command_interfaces  (unchanged)
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &position_state_[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY,  &velocity_state_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &position_command_[i]);
  }
  return interfaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// read  (called at 100 Hz)  — completely unchanged, works on any fd
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  bool got_valid_state = false;
  double p0 = position_state_[0];
  double p1 = position_state_[1];
  double p2 = position_state_[2];

  while (true) {
    std::string line = readLine();
    if (line.empty()) break;

    std::istringstream iss(line);
    std::string tag;
    double lp0, lp1, lp2;

    if (!(iss >> tag >> lp0 >> lp1 >> lp2) || tag != "STATE") {
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
        "Malformed ESP line (discarded): '%s'", line.c_str());
      continue;
    }

    p0 = lp0; p1 = lp1; p2 = lp2;
    got_valid_state = true;
  }

  if (!got_valid_state) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "No complete STATE line this cycle. Holding last known state.");
    return hardware_interface::return_type::OK;
  }

  double dt = period.seconds();
  if (dt > 0.0) {
    velocity_state_[0] = (p0 - position_state_[0]) / dt;
    velocity_state_[1] = (p1 - position_state_[1]) / dt;
    velocity_state_[2] = (p2 - position_state_[2]) / dt;
  }

  position_state_[0] = p0;
  position_state_[1] = p1;
  position_state_[2] = p2;

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write  (called at 100 Hz)  — completely unchanged, works on any fd
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  std::ostringstream cmd;
  cmd << "CMD "
      << position_command_[0] << " "
      << position_command_[1] << " "
      << position_command_[2] << "\n";

  if (!writeLine(cmd.str())) {
    RCLCPP_ERROR(logger_, "Failed to write command to ESP: %s", strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// openSerialPort  (private)  — unchanged from original
// ─────────────────────────────────────────────────────────────────────────────
bool ArmHardwareInterface::openSerialPort()
{
  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) return false;

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(serial_fd_, &tty) != 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  speed_t speed;
  switch (baud_rate_) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
      RCLCPP_WARN(logger_, "Unsupported baud rate %d, defaulting to 115200.", baud_rate_);
      speed = B115200;
  }
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// openTcpSocket  (private)
// Creates a TCP socket and connects to esp_ip_:esp_port_.
// connect() is blocking (required), then we switch to O_NONBLOCK.
// TCP_NODELAY disables Nagle's algorithm — without this, the kernel batches
// small writes and delays them up to 200 ms, destroying 100 Hz timing.
// After this returns true, serial_fd_ is a normal non-blocking fd and
// readLine()/writeLine() need zero changes.
// ─────────────────────────────────────────────────────────────────────────────
bool ArmHardwareInterface::openTcpSocket()
{
  serial_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (serial_fd_ < 0) return false;

  // Disable Nagle — critical for low-latency small-packet 100 Hz traffic
  int flag = 1;
  setsockopt(serial_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(esp_port_);
  if (inet_pton(AF_INET, esp_ip_.c_str(), &addr.sin_addr) <= 0) {
    RCLCPP_ERROR(logger_, "Invalid IP address: %s", esp_ip_.c_str());
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // connect() must be blocking — set O_NONBLOCK only AFTER it succeeds
  if (::connect(serial_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Switch to non-blocking so readLine() behaves like the serial version
  int flags = fcntl(serial_fd_, F_GETFL, 0);
  fcntl(serial_fd_, F_SETFL, flags | O_NONBLOCK);

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// writeLine  (private)  — unchanged, ::write() works on any fd
// ─────────────────────────────────────────────────────────────────────────────
bool ArmHardwareInterface::writeLine(const std::string & line)
{
  ssize_t n = ::write(serial_fd_, line.c_str(), line.size());
  return n == static_cast<ssize_t>(line.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// readLine  (private)  — unchanged, ::read() works on any fd
// ─────────────────────────────────────────────────────────────────────────────
std::string ArmHardwareInterface::readLine()
{
  char c;
  while (true) {
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      RCLCPP_ERROR(logger_, "Read error: %s", strerror(errno));
      break;
    }
    if (n == 0) break;
    if (c == '\n') {
      std::string line = serial_buffer_;
      serial_buffer_.clear();
      return line;
    }
    serial_buffer_ += c;
  }
  return "";
}


}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  arm_hardware_interface::ArmHardwareInterface,
  hardware_interface::SystemInterface
)
