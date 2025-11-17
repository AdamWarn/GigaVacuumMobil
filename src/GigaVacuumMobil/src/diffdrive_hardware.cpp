#include "GigaVacuumMobil/diffdrive_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>

// For serial communication
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace GigaVacuumMobil
{
hardware_interface::CallbackReturn DiffDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state and command variables
  left_wheel_position_ = 0.0;
  left_wheel_velocity_ = 0.0;
  right_wheel_position_ = 0.0;
  right_wheel_velocity_ = 0.0;
  left_wheel_command_ = 0.0;
  right_wheel_command_ = 0.0;

  left_encoder_ticks_ = 0;
  right_encoder_ticks_ = 0;
  prev_left_encoder_ticks_ = 0;
  prev_right_encoder_ticks_ = 0;

  // Get parameters from URDF
  device_ = info_.hardware_parameters["device"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);
  enc_ticks_per_rev_ = std::stod(info_.hardware_parameters["enc_ticks_per_rev"]);

  // Check if we should use fake hardware (for simulation)
  use_fake_hardware_ = false;
  if (info_.hardware_parameters.find("fake_hardware") != info_.hardware_parameters.end()) {
    use_fake_hardware_ = (info_.hardware_parameters["fake_hardware"] == "true");
  }

  // Verify we have the correct number of joints
  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveHardware"),
      "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

    // Initialize PID controllers with default gains (you should tune these!)\n  control_toolbox::Pid::Gains pid_gains(\n    control_toolbox::Pid::AntiWindupStrategy::CONDITIONAL_INTEGRATION);\n  pid_gains.p_gain_ = 10.0;\n  pid_gains.i_gain_ = 0.1;\n  pid_gains.d_gain_ = 0.5;\n  pid_gains.i_max_ = 100.0;\n  pid_gains.i_min_ = -100.0;\n\n  left_pid_.set_gains(pid_gains);\n  right_pid_.set_gains(pid_gains);

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveHardware"),
    "Successfully initialized DiffDriveHardware (fake_hardware=%s)",
    use_fake_hardware_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Configuring...");

  if (!use_fake_hardware_) {
    if (!openSerialPort()) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveHardware"),
        "Failed to open serial port %s", device_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("DiffDriveHardware"),
      "Serial port %s opened successfully", device_.c_str());
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("DiffDriveHardware"),
      "Using fake hardware - no serial connection");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_wheel_position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_wheel_position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_command_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_command_));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Activating...");

  // Reset commands and PIDs
  left_wheel_command_ = 0.0;
  right_wheel_command_ = 0.0;
  left_pid_.reset();
  right_pid_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Deactivating...");

  // Stop the motors
  if (!use_fake_hardware_) {
    sendPWM(0, 0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (use_fake_hardware_) {
    // In simulation, just simulate the robot based on commands
    simulateRobot(period);
    return hardware_interface::return_type::OK;
  }

  // Read encoder values from Arduino
  long left_ticks, right_ticks;
  if (!readEncoders(left_ticks, right_ticks)) {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"), "Failed to read encoders");
    return hardware_interface::return_type::ERROR;
  }

  left_encoder_ticks_ = left_ticks;
  right_encoder_ticks_ = right_ticks;

  // Calculate delta ticks
  long left_delta = left_encoder_ticks_ - prev_left_encoder_ticks_;
  long right_delta = right_encoder_ticks_ - prev_right_encoder_ticks_;

  prev_left_encoder_ticks_ = left_encoder_ticks_;
  prev_right_encoder_ticks_ = right_encoder_ticks_;

  // Convert ticks to radians
  double left_delta_rad = (left_delta / enc_ticks_per_rev_) * 2.0 * M_PI;
  double right_delta_rad = (right_delta / enc_ticks_per_rev_) * 2.0 * M_PI;

  // Update position
  left_wheel_position_ += left_delta_rad;
  right_wheel_position_ += right_delta_rad;

  // Calculate velocity (rad/s)
  double dt = period.seconds();
  if (dt > 0.0) {
    left_wheel_velocity_ = left_delta_rad / dt;
    right_wheel_velocity_ = right_delta_rad / dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (use_fake_hardware_) {
    // In simulation, we don't write anything
    return hardware_interface::return_type::OK;
  }

  // Compute PID control for left wheel
  double left_error = left_wheel_command_ - left_wheel_velocity_;
  double left_effort = left_pid_.compute_command(left_error, period.nanoseconds());

  // Compute PID control for right wheel
  double right_error = right_wheel_command_ - right_wheel_velocity_;
  double right_effort = right_pid_.compute_command(right_error, period.nanoseconds());

  // Convert effort to PWM (-255 to 255)
  // This scaling factor needs to be tuned for your specific motors
  const double effort_to_pwm = 25.0;
  int left_pwm = static_cast<int>(std::clamp(left_effort * effort_to_pwm, -255.0, 255.0));
  int right_pwm = static_cast<int>(std::clamp(right_effort * effort_to_pwm, -255.0, 255.0));

  // Send PWM to Arduino
  if (!sendPWM(left_pwm, right_pwm)) {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"), "Failed to send PWM");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool DiffDriveHardware::openSerialPort()
{
  serial_port_ = open(device_.c_str(), O_RDWR | O_NOCTTY);

  if (serial_port_ < 0) {
    return false;
  }

  // Configure serial port
  struct termios tty;
  if (tcgetattr(serial_port_, &tty) != 0) {
    close(serial_port_);
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (baud_rate_) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    default: speed = B57600;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1 mode
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = timeout_ms_ / 100;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
    close(serial_port_);
    return false;
  }

  // Flush any existing data
  tcflush(serial_port_, TCIOFLUSH);

  return true;
}

void DiffDriveHardware::closeSerialPort()
{
  if (serial_port_ >= 0) {
    close(serial_port_);
    serial_port_ = -1;
  }
}

bool DiffDriveHardware::readEncoders(long &left_ticks, long &right_ticks)
{
  // Request encoder data from Arduino
  // Protocol: send 'e\n', expect response 'e{left},{right}\n'
  const char* request = "e\n";
  ::write(serial_port_, request, 2);

  // Read response
  char buffer[64];
  int n = ::read(serial_port_, buffer, sizeof(buffer) - 1);
  
  if (n <= 0) {
    return false;
  }

  buffer[n] = '\0';

  // Parse response
  if (buffer[0] == 'e') {
    if (sscanf(buffer, "e%ld,%ld", &left_ticks, &right_ticks) == 2) {
      return true;
    }
  }

  return false;
}

bool DiffDriveHardware::sendPWM(int left_pwm, int right_pwm)
{
  // Protocol: send 'p{left},{right}\n'
  char command[32];
  snprintf(command, sizeof(command), "p%d,%d\n", left_pwm, right_pwm);
  
  int n = ::write(serial_port_, command, strlen(command));
  return (n > 0);
}

void DiffDriveHardware::simulateRobot(const rclcpp::Duration & period)
{
  // Simple simulation: wheels respond instantly to commands
  double dt = period.seconds();
  
  left_wheel_velocity_ = left_wheel_command_;
  right_wheel_velocity_ = right_wheel_command_;
  
  left_wheel_position_ += left_wheel_velocity_ * dt;
  right_wheel_position_ += right_wheel_velocity_ * dt;
}

}  // namespace GigaVacuumMobil

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  GigaVacuumMobil::DiffDriveHardware, hardware_interface::SystemInterface)