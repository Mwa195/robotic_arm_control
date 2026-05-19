#include "arm_hardware_interface/hardware_translator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// POSIX serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

// String parsing
#include <sstream>
#include <stdexcept>

namespace arm_hardware_interface
{

// ─────────────────────────────────────────────────────────────────────────────
// on_init
// Called once when the plugin is loaded by the controller_manager.
// Reads parameters from the URDF <hardware> block and validates the joint
// configuration matches what we expect (3 joints, position cmd, pos+vel state).
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  // Always call the parent first — it populates this->info_
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // ── Read URDF params ──────────────────────────────────────────────────────
  // These come from <param name="serial_port">/dev/ttyUSB0</param> in the URDF.
  // info_.hardware_parameters is a std::map<std::string, std::string>.
  try {
    port_      = info_.hardware_parameters.at("serial_port");
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(logger_,
      "URDF is missing required params. Add:\n"
      "  <param name=\"serial_port\">/dev/ttyUSB0</param>\n"
      "  <param name=\"baud_rate\">115200</param>");
    return CallbackReturn::ERROR;
  }

  serial_fd_ = -1;

  // ── Validate joints ───────────────────────────────────────────────────────
  if (info_.joints.size() != 3) {
    RCLCPP_FATAL(logger_,
      "Expected 3 joints, got %zu. Check URDF <ros2_control> block.",
      info_.joints.size());
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    // Validate command interface — must have exactly one: position
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(logger_,
        "Joint '%s' must have exactly one 'position' command interface.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    // Validate state interfaces — must have position and velocity
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

  // ── Initialise data arrays ────────────────────────────────────────────────
  // Size 3, all zeros. Order matches joint_names_ which follows URDF order:
  // [0]=base_joint  [1]=shoulder_joint  [2]=elbow_joint
  position_state_.assign(3, 0.0);
  velocity_state_.assign(3, 0.0);
  position_command_.assign(3, 0.0);

  RCLCPP_INFO(logger_, "on_init OK — port: %s @ %d baud", port_.c_str(), baud_rate_);
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure
// Opens and configures the serial port. Called when transitioning to
// "configured" state. Errors here prevent activation.
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!openSerialPort()) {
    RCLCPP_ERROR(logger_, "Failed to open serial port '%s': %s",
      port_.c_str(), strerror(errno));
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Serial port %s opened successfully.", port_.c_str());
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate
// Seeds position_command_ with the current encoder reading so the arm does
// not snap on the very first write() cycle.
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Mirror current state into command — prevents jump-on-activate
  position_command_ = position_state_;

  RCLCPP_INFO(logger_, "Arm hardware interface activated.");
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate
// Closes the serial port cleanly.
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(logger_, "Serial port closed.");
  }
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
// Binds position_state_[i] and velocity_state_[i] to the framework.
// The framework gives these pointers to joint_state_broadcaster, which
// reads them every cycle and publishes /joint_states.
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

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// Binds position_command_[i] to the framework.
// arm_controller writes the trajectory waypoint into these every cycle.
// ─────────────────────────────────────────────────────────────────────────────
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
// read  (called at 100 Hz)
// Reads a "STATE t1 t2 t3\n" line from the ESP and stores the values
// in position_state_. Estimates velocity as Δpos/Δt.
// Non-fatal on parse errors — logs a warning and keeps the last known state.
// ─────────────────────────────────────────────────────────────────────────────

//old
// hardware_interface::return_type ArmHardwareInterface::read(
//   const rclcpp::Time & /*time*/,
//   const rclcpp::Duration & period)
// {
//   std::string line = readLine();

//   if (line.empty()) {
//     // Timeout or no data yet — not an error during startup
//     RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
//       "No data from ESP (timeout). Holding last known state.");
//     return hardware_interface::return_type::OK;
//   }

//   // Expected format: "STATE 1.570796 0.500000 -0.300000"
//   std::istringstream iss(line);
//   std::string tag;
//   double p0, p1, p2;

//   if (!(iss >> tag >> p0 >> p1 >> p2) || tag != "STATE") {
//     RCLCPP_WARN(logger_, "Malformed ESP response: '%s'", line.c_str());
//     return hardware_interface::return_type::OK;  // keep last known, don't crash
//   }

//   // Estimate velocity = Δposition / Δtime
//   double dt = period.seconds();
//   if (dt > 0.0) {
//     velocity_state_[0] = (p0 - position_state_[0]) / dt;
//     velocity_state_[1] = (p1 - position_state_[1]) / dt;
//     velocity_state_[2] = (p2 - position_state_[2]) / dt;
//   }

//   position_state_[0] = p0;
//   position_state_[1] = p1;
//   position_state_[2] = p2;

//   return hardware_interface::return_type::OK;
// }

hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // Drain ALL complete lines available this cycle.
  // The ESP sends STATE at 100 Hz independently; the kernel buffer accumulates
  // multiple lines between read() calls when the scheduler delays this thread.
  // Processing only the LAST valid STATE prevents stale-data lag and buffer overflow.
  bool got_valid_state = false;
  double p0 = position_state_[0];  // default: keep last known
  double p1 = position_state_[1];
  double p2 = position_state_[2];

  while (true) {
    std::string line = readLine();
    if (line.empty()) break;  // no more complete lines this cycle

    std::istringstream iss(line);
    std::string tag;
    double lp0, lp1, lp2;

    if (!(iss >> tag >> lp0 >> lp1 >> lp2) || tag != "STATE") {
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
        "Malformed ESP line (discarded): '%s'", line.c_str());
      continue;  // skip, keep draining
    }

    p0 = lp0; p1 = lp1; p2 = lp2;
    got_valid_state = true;
  }

  if (!got_valid_state) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "No complete STATE line this cycle. Holding last known state.");
    return hardware_interface::return_type::OK;
  }

  // Estimate velocity = Δposition / Δtime (from freshest STATE only)
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
// write  (called at 100 Hz)
// Reads position_command_[0..2] (set by arm_controller) and sends
// "CMD t1 t2 t3\n" to the ESP.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Build the command string
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
// openSerialPort  (private)
// Opens port_ and configures termios:
//   8 data bits, no parity, 1 stop bit (8N1)
//   Raw mode (no line buffering, no echo)
//   VMIN=0 VTIME=1 → non-blocking with 100 ms timeout
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

  // Set baud rate
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

  // 8N1
  tty.c_cflag &= ~PARENB;   // no parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;        // 8 data bits

  // Hardware flow control off
  tty.c_cflag &= ~CRTSCTS;

  // Enable receiver, ignore modem control lines
  tty.c_cflag |= (CLOCAL | CREAD);

  // Raw input — no line processing, no echo, no signals
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

  // Raw output
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  // Input flags — disable software flow control, disable special byte handling
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // Non-blocking read with 100 ms timeout
  // VMIN=0: return immediately if no data
  // VTIME=1: wait up to 100 ms (value is in tenths of a second)
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
// writeLine  (private)
// Writes the entire string to the serial file descriptor.
// Returns false if the system write() fails.
// ─────────────────────────────────────────────────────────────────────────────
bool ArmHardwareInterface::writeLine(const std::string & line)
{
  ssize_t n = ::write(serial_fd_, line.c_str(), line.size());
  return n == static_cast<ssize_t>(line.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// readLine  (private)
// Reads one character at a time until '\n' or timeout.
// Returns the line without the trailing '\n'.
// Returns empty string on timeout or serial error.
// ─────────────────────────────────────────────────────────────────────────────

// old
// std::string ArmHardwareInterface::readLine()
// {
//   std::string result;
//   char c;

//   while (true) {
//     ssize_t n = ::read(serial_fd_, &c, 1);

//     if (n < 0) {
//       // Real error
//       RCLCPP_ERROR(logger_, "Serial read error: %s", strerror(errno));
//       return "";
//     }
//     if (n == 0) {
//       // Timeout (VTIME expired with no data)
//       return "";
//     }
//     if (c == '\n') {
//       break;
//     }
//     result += c;
//   }

//   return result;
// }

// old 2
// std::string ArmHardwareInterface::readLine()
// {
//   // Drain all available bytes into the persistent buffer
//   char c;
//   while (true) {
//     ssize_t n = ::read(serial_fd_, &c, 1);
//     if (n <= 0) break;  // nothing left right now
//     if (c == '\n') {
//       std::string line = serial_buffer_;
//       serial_buffer_.clear();
//       return line;
//     }
//     serial_buffer_ += c;
//   }
//   return "";  // no complete line yet — caller keeps last known state
// }

std::string ArmHardwareInterface::readLine()
{
  char c;
  while (true) {
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // no data right now
      RCLCPP_ERROR(logger_, "Serial read error: %s", strerror(errno));
      break;
    }
    if (n == 0) break;  // shouldn't happen with O_NONBLOCK but guard it
    if (c == '\n') {
      std::string line = serial_buffer_;
      serial_buffer_.clear();
      return line;
    }
    serial_buffer_ += c;
  }
  return "";  // no complete line yet
}


}  // namespace arm_hardware_interface

// ─────────────────────────────────────────────────────────────────────────────
// pluginlib export — must be OUTSIDE any namespace, at the bottom of the file
// First arg:  your fully-qualified class name
// Second arg: the base class from ros2_control
// ─────────────────────────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  arm_hardware_interface::ArmHardwareInterface,
  hardware_interface::SystemInterface
)