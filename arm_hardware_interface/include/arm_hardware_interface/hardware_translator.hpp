#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include <string>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace arm_hardware_interface
{

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardwareInterface)

  // ── Lifecycle ─────────────────────────────────────────────────────────────

  // Called once on plugin load.
  // Reads transport ("serial" or "wifi") from URDF <param> tags.
  // For serial: reads serial_port and baud_rate.
  // For wifi:   reads esp_ip and esp_port.
  // Validates that the URDF declares exactly 3 joints with the expected
  // command/state interfaces.
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Called when transitioning to "configured".
  // Opens serial port OR TCP socket depending on transport_.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  // Called when transitioning to "active" (arm ready to receive commands).
  // Seeds position_command_ with the current position_state_ so the arm
  // does not jump on first write().
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Called on shutdown or error. Closes the fd (serial or socket).
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface exports ─────────────────────────────────────────────────────

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Control loop (100 Hz) ─────────────────────────────────────────────────

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ── Transport helpers ─────────────────────────────────────────────────────

  // Opens /dev/ttyUSBx at baud_rate_, configures termios 8N1 raw mode.
  bool openSerialPort();

  // Opens a non-blocking TCP socket to esp_ip_:esp_port_.
  // Sets TCP_NODELAY to eliminate Nagle buffering (critical for 100 Hz).
  bool openTcpSocket();

  // Writes a complete line (including '\n') to serial_fd_.
  bool writeLine(const std::string & line);

  // Reads characters until '\n'. Returns line without '\n', or "" on timeout.
  // Works identically for serial fd and TCP socket fd.
  std::string readLine();
  std::string serial_buffer_;

  // ── Joint data ────────────────────────────────────────────────────────────

  // Ordered: [0]=base_joint  [1]=shoulder_joint  [2]=elbow_joint
  std::vector<double> position_state_;
  std::vector<double> velocity_state_;
  std::vector<double> position_command_;
  std::vector<std::string> joint_names_;

  // ── Transport config ──────────────────────────────────────────────────────

  // "serial" or "wifi" — set by URDF param "transport"
  std::string transport_;

  // Serial params (used when transport_ == "serial")
  std::string port_;      // e.g. "/dev/ttyUSB0"
  int baud_rate_;         // e.g. 115200

  // WiFi/TCP params (used when transport_ == "wifi")
  std::string esp_ip_;    // e.g. "192.168.1.105"
  int esp_port_;          // e.g. 8888

  // Unified file descriptor — works for both serial and TCP socket
  int serial_fd_;

  // ── Logging ───────────────────────────────────────────────────────────────

  rclcpp::Logger logger_ = rclcpp::get_logger("ArmHardwareInterface");
};

}  // namespace arm_hardware_interface