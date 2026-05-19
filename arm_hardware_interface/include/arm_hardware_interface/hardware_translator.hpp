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
  // Reads serial_port and baud_rate from URDF <param> tags.
  // Validates that the URDF declares exactly 3 joints with the expected
  // command/state interfaces.
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Called when transitioning to "configured".
  // Opens and configures the serial port (8N1, no flow control, raw mode).
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  // Called when transitioning to "active" (arm ready to receive commands).
  // Seeds position_command_ with the current position_state_ so the arm
  // does not jump on first write().
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Called on shutdown or error. Closes the serial port.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface exports ─────────────────────────────────────────────────────

  // Registers position_state_[i] and velocity_state_[i] with the framework
  // under names "base_joint/position", "shoulder_joint/position", etc.
  // joint_state_broadcaster reads these to publish /joint_states.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Registers position_command_[i] with the framework.
  // arm_controller writes the desired position into these every cycle.
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Control loop (100 Hz) ─────────────────────────────────────────────────

  // Sends "CMD t1 t2 t3\n" to the Pico over UART then reads back
  // "STATE t1 t2 t3\n" and stores the values in position_state_.
  // Velocity is estimated as (Δposition / Δtime).
  // Returns ERROR and logs a warning if the serial exchange fails,
  // but does NOT crash — the controller manager handles retries.
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Reads position_command_[0..2] (written by arm_controller) and
  // sends "CMD t1 t2 t3\n" to the Pico.
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ── Serial helpers ────────────────────────────────────────────────────────

  // Opens port_ at baud_rate_, configures termios for 8N1 raw mode,
  // sets VMIN=0 VTIME=1 (100 ms timeout). Returns true on success.
  bool openSerialPort();

  // Writes a complete line (including '\n') to serial_fd_.
  // Returns false if write() returns an error.
  bool writeLine(const std::string & line);

  // Reads characters until '\n' or timeout. Returns the line without '\n'.
  // Returns empty string on timeout or error.
  std::string readLine();
  std::string serial_buffer_;

  // ── Joint data ────────────────────────────────────────────────────────────

  // Ordered: [0]=base_joint  [1]=shoulder_joint  [2]=elbow_joint
  // These arrays are bound to the framework by export_*_interfaces().
  // read()  → writes into position_state_ and velocity_state_
  // write() → reads from position_command_
  std::vector<double> position_state_;    // radians, from Pico encoder feedback
  std::vector<double> velocity_state_;    // rad/s,   estimated from Δpos/Δt
  std::vector<double> position_command_;  // radians, set by arm_controller

  // Joint names in the same order as the arrays above.
  // Filled from info.joints in on_init() so the order matches the URDF.
  std::vector<std::string> joint_names_;

  // ── Serial port config ────────────────────────────────────────────────────

  std::string port_;      // e.g. "/dev/ttyUSB0" — read from URDF param
  int baud_rate_;         // e.g. 115200         — read from URDF param
  int serial_fd_;         // POSIX file descriptor, -1 when not open

  // ── Logging ───────────────────────────────────────────────────────────────

  rclcpp::Logger logger_ = rclcpp::get_logger("ArmHardwareInterface");
};

}  // namespace arm_hardware_interface