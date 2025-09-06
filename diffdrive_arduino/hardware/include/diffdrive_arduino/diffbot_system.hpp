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

// ROS2 Service ve Message include'ları
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

// Arduino communication ve wheel include'ları
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
  bool enable_reverse_buzzer = true;     // Geri gitme buzzer'ını aktif et/deaktif et
  bool enable_servo_control = true;      // Servo kontrolünü aktif et/deaktif et
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

  // =================== BUZZER KONTROLÜ ===================
  DIFFDRIVE_ARDUINO_PUBLIC
  void set_manual_buzzer(bool active);

  DIFFDRIVE_ARDUINO_PUBLIC
  void enable_reverse_buzzer(bool enable);

  DIFFDRIVE_ARDUINO_PUBLIC
  bool is_buzzer_active() const;

  // =================== SES KONTROLÜ ===================
  DIFFDRIVE_ARDUINO_PUBLIC
  void play_sound_1();

  DIFFDRIVE_ARDUINO_PUBLIC
  void play_sound_2();

  DIFFDRIVE_ARDUINO_PUBLIC
  void play_sound(int sound_number);

  // =================== SERVO KONTROLÜ ===================
  DIFFDRIVE_ARDUINO_PUBLIC
  void trigger_servo_movement();

  DIFFDRIVE_ARDUINO_PUBLIC
  void trigger_servo(int servo_index);

  DIFFDRIVE_ARDUINO_PUBLIC
  bool is_servo_available() const;

private:
  // =================== CORE HARDWARE ===================
  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  
  // =================== ROS2 INTERFACE ===================
  rclcpp::Node::SharedPtr node_;
  
  // =================== BUZZER SERVICES & PUBLISHERS ===================
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr buzzer_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr buzzer_status_publisher_;
  
  // Buzzer durumu yönetimi
  bool buzzer_reverse_active_ = false;  // Geri gitme buzzer durumu
  bool buzzer_manual_active_ = false;   // Manuel buzzer durumu
  bool buzzer_obstacle_active_ = false; // Engel buzzer durumu
  bool buzzer_active_ = false;          // Genel buzzer durumu
  
  // =================== SES SERVICES ===================
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sound1_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sound2_service_;

  // =================== SERVO SERVICES & PUBLISHERS ===================
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servo_trigger_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_status_publisher_;
  
  // Servo durumu yönetimi
  bool servo_triggered_ = false;        // Servo tetiklenme durumu
  
  // =================== PRIVATE METHODS ===================
  
  // ROS Interface kurulum
  void setup_ros_interfaces();
  
  // Buzzer yönetimi
  void update_buzzer_state();
  void check_reverse_condition();
  void publish_buzzer_status();
  
  // Servo yönetimi
  void publish_servo_status(bool triggered);
  
  // =================== SERVICE CALLBACKS ===================
  
  // Buzzer service callback
  void buzzer_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  
  // Ses service callbacks
  void sound1_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  
  void sound2_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Servo service callback
  void servo_trigger_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

}  // namespace diffdrive_arduino

#endif  // DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_