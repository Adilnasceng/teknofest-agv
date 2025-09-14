// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <iomanip>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = ::std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = ::std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = ::std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.left_enc_counts_per_rev = ::std::stoi(info_.hardware_parameters["left_enc_counts_per_rev"]);
  cfg_.right_enc_counts_per_rev = ::std::stoi(info_.hardware_parameters["right_enc_counts_per_rev"]);
  
  // Buzzer eşik değerini config'den oku
  if (info_.hardware_parameters.count("reverse_speed_threshold") > 0)
  {
    cfg_.reverse_speed_threshold = ::std::stod(info_.hardware_parameters["reverse_speed_threshold"]);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Reverse speed threshold set to: %.3f", cfg_.reverse_speed_threshold);
  }
  else 
  {
    cfg_.reverse_speed_threshold = -0.1; // Default değer
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "reverse_speed_threshold not found in config, using default: %.3f", cfg_.reverse_speed_threshold);
  }

  // Geri gitme buzzer'ı kontrol parametresi
  if (info_.hardware_parameters.count("enable_reverse_buzzer") > 0)
  {
    cfg_.enable_reverse_buzzer = (info_.hardware_parameters["enable_reverse_buzzer"] == "true");
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Reverse buzzer enabled: %s", cfg_.enable_reverse_buzzer ? "true" : "false");
  }
  else 
  {
    cfg_.enable_reverse_buzzer = true; // Default aktif
  }

  // Servo control parametresi
  if (info_.hardware_parameters.count("enable_servo_control") > 0)
  {
    cfg_.enable_servo_control = (info_.hardware_parameters["enable_servo_control"] == "true");
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo control enabled: %s", cfg_.enable_servo_control ? "true" : "false");
  }
  else 
  {
    cfg_.enable_servo_control = true; // Default aktif
  }

  // Battery read interval parametresi
  if (info_.hardware_parameters.count("battery_read_interval") > 0)
  {
    cfg_.battery_read_interval = ::std::stod(info_.hardware_parameters["battery_read_interval"]);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Battery read interval set to: %.1f seconds", cfg_.battery_read_interval);
  }
  else 
  {
    cfg_.battery_read_interval = 5.0; // Default değer
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Using default battery read interval: %.1f seconds", cfg_.battery_read_interval);
  }
  
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = ::std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = ::std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = ::std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = ::std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.left_enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.right_enc_counts_per_rev);

  // Buzzer durumunu başlat
  buzzer_reverse_active_ = false;
  buzzer_manual_active_ = false;
  buzzer_obstacle_active_ = false;
  buzzer_active_ = false;

  // Servo durumunu başlat
  servo_triggered_ = false;

  // Battery durumunu başlat
  battery_voltage_ = 0.0f;
  battery_percentage_ = 0.0f;
  battery_available_ = false;

  // ROS interfaces kurulumu
  setup_ros_interfaces();

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void DiffDriveArduinoHardware::setup_ros_interfaces()
{
  // ROS node oluştur
  node_ = rclcpp::Node::make_shared("diffbot_hardware_interface");
  
  // Buzzer control service
  buzzer_service_ = node_->create_service<std_srvs::srv::SetBool>(
    "set_buzzer_state",
    std::bind(&DiffDriveArduinoHardware::buzzer_service_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Buzzer status publisher
  buzzer_status_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
    "buzzer_status", 10);

  // Ses kontrolü servisleri
  sound1_service_ = node_->create_service<std_srvs::srv::SetBool>(
    "play_sound_1",
    std::bind(&DiffDriveArduinoHardware::sound1_service_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  sound2_service_ = node_->create_service<std_srvs::srv::SetBool>(
    "play_sound_2",
    std::bind(&DiffDriveArduinoHardware::sound2_service_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Servo servisi ve publisher
  servo_trigger_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "trigger_servo",
    std::bind(&DiffDriveArduinoHardware::servo_trigger_service_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  servo_status_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
    "servo_status", 10);

  // Battery servisi ve publisher
  battery_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "get_battery_status",
    std::bind(&DiffDriveArduinoHardware::battery_service_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  battery_status_publisher_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery_status", 10);

  // Battery timing başlatma
  battery_read_interval_ = cfg_.battery_read_interval;
  last_battery_read_ = rclcpp::Clock().now();

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
              "ROS interfaces setup complete - Services: /set_buzzer_state, /play_sound_1, /play_sound_2, /trigger_servo, /get_battery_status, Topics: /buzzer_status, /servo_status, /battery_status");
}

void DiffDriveArduinoHardware::buzzer_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  buzzer_obstacle_active_ = request->data;
  update_buzzer_state();
  
  response->success = true;
  response->message = request->data ? "Buzzer activated for obstacle" : "Buzzer deactivated";
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
              "Buzzer service called: %s", response->message.c_str());
}

void DiffDriveArduinoHardware::sound1_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) // True ise sesi çal
  {
    play_sound_1();
    response->success = true;
    response->message = "Sound 1 played successfully";
  }
  else
  {
    response->success = true;
    response->message = "Sound 1 service called but not played (data=false)";
  }
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
              "Sound 1 service called: %s", response->message.c_str());
}

void DiffDriveArduinoHardware::sound2_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) // True ise sesi çal
  {
    play_sound_2();
    response->success = true;
    response->message = "Sound 2 played successfully";
  }
  else
  {
    response->success = true;
    response->message = "Sound 2 service called but not played (data=false)";
  }
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
              "Sound 2 service called: %s", response->message.c_str());
}

void DiffDriveArduinoHardware::servo_trigger_service_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // Trigger service'inin request'i boş olabilir
  
  if (comms_.connected() && cfg_.enable_servo_control)
  {
    trigger_servo_movement();
    
    response->success = true;
    response->message = "Servo triggered successfully - Moving to 90 degrees for 15 seconds";
    
    // Servo status publish et
    publish_servo_status(true);
    
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo service called: %s", response->message.c_str());
  }
  else if (!comms_.connected())
  {
    response->success = false;
    response->message = "Arduino connection not available";
    
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                 "Servo service failed: %s", response->message.c_str());
  }
  else
  {
    response->success = false;
    response->message = "Servo control is disabled in configuration";
    
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo service failed: %s", response->message.c_str());
  }
}

void DiffDriveArduinoHardware::battery_service_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // Trigger service'inin request'i kullanılmıyor
  
  if (comms_.connected())
  {
    update_battery_data();
    
    if (battery_available_)
    {
      response->success = true;
      
      std::stringstream ss;
      ss << "Battery: " << std::fixed << std::setprecision(2) 
         << battery_voltage_ << "V (" << std::setprecision(1) 
         << battery_percentage_ << "%)";
      response->message = ss.str();
      
      // Anında battery status publish et
      publish_battery_status();
      
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                  "Battery service called: %s", response->message.c_str());
    }
    else
    {
      response->success = false;
      response->message = "Battery data not available or invalid";
      
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                  "Battery service failed: %s", response->message.c_str());
    }
  }
  else
  {
    response->success = false;
    response->message = "Arduino connection not available";
    
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                 "Battery service failed: %s", response->message.c_str());
  }
}

void DiffDriveArduinoHardware::publish_buzzer_status()
{
  auto msg = std_msgs::msg::Bool();
  msg.data = buzzer_active_;
  buzzer_status_publisher_->publish(msg);
}

void DiffDriveArduinoHardware::publish_servo_status(bool triggered)
{
  auto msg = std_msgs::msg::Bool();
  msg.data = triggered;
  servo_status_publisher_->publish(msg);
  
  if (triggered)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo status published: TRIGGERED");
  }
}

void DiffDriveArduinoHardware::publish_battery_status()
{
  auto battery_msg = sensor_msgs::msg::BatteryState();
  
  // Temel bilgiler
  battery_msg.header.stamp = rclcpp::Clock().now();
  battery_msg.header.frame_id = "base_link";
  
  if (battery_available_)
  {
    battery_msg.voltage = battery_voltage_;
    battery_msg.percentage = battery_percentage_ / 100.0f; // ROS standartı 0-1 arası
    battery_msg.present = true;
    
    // Basit durum belirleme
    battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    battery_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    
    // Kapasite bilgileri
    battery_msg.design_capacity = 100.0f; // % cinsinden maksimum kapasite
    battery_msg.capacity = battery_percentage_; // Mevcut kapasite
  }
  else
  {
    battery_msg.voltage = 0.0f;
    battery_msg.percentage = 0.0f;
    battery_msg.present = false;
    battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  }
  
  battery_status_publisher_->publish(battery_msg);
}

::std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  ::std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

::std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  ::std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  // Buzzer'ı kapat
  buzzer_active_ = false;
  comms_.set_buzzer_state(false);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Enkoder okumalarını aynı anda yap
  int left_enc_temp, right_enc_temp;
  comms_.read_encoder_values(left_enc_temp, right_enc_temp);
  
  // Değerleri aynı anda ata
  wheel_l_.enc = left_enc_temp;
  wheel_r_.enc = right_enc_temp;

  double delta_seconds = period.seconds();

  // Sol tekerlek hesaplaması
  double pos_prev_l = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev_l) / delta_seconds;

  // Sağ tekerlek hesaplaması
  double pos_prev_r = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev_r) / delta_seconds;

  // Battery durumunu düzenli aralıklarla oku
  rclcpp::Time current_time = rclcpp::Clock().now();
  double elapsed_seconds = (current_time - last_battery_read_).seconds();
  
  if (elapsed_seconds >= battery_read_interval_)
  {
    update_battery_data();
    publish_battery_status();
    last_battery_read_ = current_time;
  }

  // ROS spin for service calls
  if (node_)
  {
    rclcpp::spin_some(node_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  
  // Geri gitme kontrolü (sadece aktifse)
  if (cfg_.enable_reverse_buzzer)
  {
    check_reverse_condition();
  }
  
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
  
  // Buzzer status publish et
  publish_buzzer_status();
  
  return hardware_interface::return_type::OK;
}

void DiffDriveArduinoHardware::set_manual_buzzer(bool active)
{
  if (buzzer_manual_active_ != active)
  {
    buzzer_manual_active_ = active;
    update_buzzer_state();
    
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Manual buzzer set to: %s", active ? "ON" : "OFF");
  }
}

void DiffDriveArduinoHardware::enable_reverse_buzzer(bool enable)
{
  cfg_.enable_reverse_buzzer = enable;
  
  if (!enable)
  {
    // Geri gitme buzzer'ını deaktif et
    buzzer_reverse_active_ = false;
    update_buzzer_state();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
              "Reverse buzzer %s", enable ? "enabled" : "disabled");
}

bool DiffDriveArduinoHardware::is_buzzer_active() const
{
  return buzzer_active_;
}

void DiffDriveArduinoHardware::update_buzzer_state()
{
  bool should_be_active = (buzzer_reverse_active_ || buzzer_manual_active_ || buzzer_obstacle_active_);
  
  if (should_be_active != buzzer_active_)
  {
    buzzer_active_ = should_be_active;
    comms_.set_buzzer_state(buzzer_active_);
    
    if (buzzer_active_)
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                  "Buzzer ON - Reverse: %s, Manual: %s, Obstacle: %s", 
                  buzzer_reverse_active_ ? "ON" : "OFF",
                  buzzer_manual_active_ ? "ON" : "OFF",
                  buzzer_obstacle_active_ ? "ON" : "OFF");
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Buzzer OFF");
    }
  }
}

void DiffDriveArduinoHardware::check_reverse_condition()
{
  bool is_reversing = (wheel_l_.cmd < cfg_.reverse_speed_threshold && 
                      wheel_r_.cmd < cfg_.reverse_speed_threshold);
  
  if (buzzer_reverse_active_ != is_reversing)
  {
    buzzer_reverse_active_ = is_reversing;
    update_buzzer_state();
  }
}

void DiffDriveArduinoHardware::play_sound_1()
{
  if (comms_.connected())
  {
    comms_.play_sound_1();
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Playing sound 1");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cannot play sound 1 - not connected");
  }
}

void DiffDriveArduinoHardware::play_sound_2()
{
  if (comms_.connected())
  {
    comms_.play_sound_2();
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Playing sound 2");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cannot play sound 2 - not connected");
  }
}

void DiffDriveArduinoHardware::play_sound(int sound_number)
{
  if (comms_.connected())
  {
    if (sound_number >= 1 && sound_number <= 2)
    {
      comms_.play_sound(sound_number);
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Playing sound %d", sound_number);
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), "Invalid sound number: %d (valid range: 1-2)", sound_number);
    }
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cannot play sound %d - not connected", sound_number);
  }
}

void DiffDriveArduinoHardware::trigger_servo_movement()
{
  if (comms_.connected() && cfg_.enable_servo_control)
  {
    comms_.trigger_servo_movement();
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo tetiklendi - 90 dereceye gidip 15 saniye bekleyecek");
  }
  else if (!comms_.connected())
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo tetiklenemedi - Arduino bağlantısı yok");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo tetiklenemedi - Servo kontrolü deaktif");
  }
}

void DiffDriveArduinoHardware::trigger_servo(int servo_index)
{
  if (comms_.connected() && cfg_.enable_servo_control)
  {
    comms_.trigger_servo(servo_index);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo %d tetiklendi", servo_index);
  }
  else if (!comms_.connected())
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo %d tetiklenemedi - Arduino bağlantısı yok", servo_index);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Servo %d tetiklenemedi - Servo kontrolü deaktif", servo_index);
  }
}

bool DiffDriveArduinoHardware::is_servo_available() const
{
  return (comms_.connected() && cfg_.enable_servo_control);
}

void DiffDriveArduinoHardware::read_battery_status()
{
  update_battery_data();
}

bool DiffDriveArduinoHardware::get_battery_info(float &voltage, float &percentage)
{
  if (comms_.connected())
  {
    update_battery_data();
    voltage = battery_voltage_;
    percentage = battery_percentage_;
    return battery_available_;
  }
  else
  {
    voltage = 0.0f;
    percentage = 0.0f;
    return false;
  }
}

bool DiffDriveArduinoHardware::is_battery_available() const
{
  return (comms_.connected() && battery_available_);
}

void DiffDriveArduinoHardware::set_battery_read_interval(double interval_seconds)
{
  battery_read_interval_ = interval_seconds;
  cfg_.battery_read_interval = interval_seconds;
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
              "Battery read interval changed to: %.1f seconds", interval_seconds);
}

double DiffDriveArduinoHardware::get_battery_read_interval() const
{
  return battery_read_interval_;
}

void DiffDriveArduinoHardware::update_battery_data()
{
  if (!comms_.connected())
  {
    battery_available_ = false;
    return;
  }
  
  try
  {
    float voltage, percentage;
    battery_available_ = comms_.get_battery_status(voltage, percentage);
    
    if (battery_available_)
    {
      battery_voltage_ = voltage;
      battery_percentage_ = percentage;
      
      RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                   "Battery: %.2fV (%.1f%%)", battery_voltage_, battery_percentage_);
    }
  }
  catch (const std::exception& e)
  {
    battery_available_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                 "Battery read error: %s", e.what());
  }
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)