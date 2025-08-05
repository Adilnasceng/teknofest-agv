#ifndef DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diffdrive_arduino/visibility_control.h"

#include "diffdrive_arduino/arduino_comms.hpp"
#include "diffdrive_arduino/wheel.hpp"

namespace diffdrive_arduino
{

class DiffDriveArduinoHardware : public hardware_interface::SystemInterface
{

struct Config
{
  ::std::string left_wheel_name = "";
  ::std::string right_wheel_name = "";
  float loop_rate = 0.0;
  ::std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int left_enc_counts_per_rev = 0;
  int right_enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
  double reverse_speed_threshold = -0.1; // Geri gitme eşiği (m/s)
  bool enable_reverse_buzzer = true; // Geri gitme buzzer'ını aktif et/deaktif et
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveArduinoHardware);

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  ::std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  ::std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Basit buzzer kontrolü için public fonksiyonlar
  DIFFDRIVE_ARDUINO_PUBLIC
  void set_manual_buzzer(bool active);

  DIFFDRIVE_ARDUINO_PUBLIC
  void enable_reverse_buzzer(bool enable);

  DIFFDRIVE_ARDUINO_PUBLIC
  bool is_buzzer_active() const;

private:
  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  
  // Basit buzzer durumu yönetimi
  bool buzzer_reverse_active_ = false;  // Geri gitme buzzer durumu
  bool buzzer_manual_active_ = false;   // Manuel buzzer durumu
  bool buzzer_active_ = false;          // Genel buzzer durumu
  
  void update_buzzer_state();
  void check_reverse_condition();
};

}  // namespace diffdrive_arduino

#endif  // DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_