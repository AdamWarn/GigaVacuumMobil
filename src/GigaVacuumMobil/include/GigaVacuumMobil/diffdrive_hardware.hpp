#ifndef GIGAVACUUMMOBIL_DIFFDRIVE_HARDWARE_HPP
#define GIGAVACUUMMOBIL_DIFFDRIVE_HARDWARE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "control_toolbox/pid.hpp"

namespace GigaVacuumMobil
{
class DiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication
  int serial_port_;
  std::string device_;
  int baud_rate_;
  int timeout_ms_;
  bool use_fake_hardware_;

  // Encoder parameters
  double enc_ticks_per_rev_;

  // State storage (position and velocity in radians and rad/s)
  double left_wheel_position_;
  double left_wheel_velocity_;
  double right_wheel_position_;
  double right_wheel_velocity_;

  // Command storage (target velocity in rad/s)
  double left_wheel_command_;
  double right_wheel_command_;

  // Previous encoder tick counts for delta calculation
  long left_encoder_ticks_;
  long right_encoder_ticks_;
  long prev_left_encoder_ticks_;
  long prev_right_encoder_ticks_;

  // PID Controllers
  control_toolbox::Pid left_pid_;
  control_toolbox::Pid right_pid_;

  // Helper functions
  bool openSerialPort();
  void closeSerialPort();
  bool readEncoders(long &left_ticks, long &right_ticks);
  bool sendPWM(int left_pwm, int right_pwm);
  
  // Simulation helpers
  void simulateRobot(const rclcpp::Duration & period);
};

}  // namespace GigaVacuumMobil

#endif  // GIGAVACUUMMOBIL_DIFFDRIVE_HARDWARE_HPP