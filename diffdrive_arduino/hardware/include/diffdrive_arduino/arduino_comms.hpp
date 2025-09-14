#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      ::std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << ::std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const ::std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  ::std::string send_msg(const ::std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    ::std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        ::std::cerr << "The ReadByte() call has timed out." << ::std::endl ;
    }

    if (print_output)
    {
      ::std::cout << "Sent: " << msg_to_send << " Recv: " << response << ::std::endl;
    }

    return response;
  }

  void send_empty_msg()
  {
    ::std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    ::std::string response = send_msg("e\r");

    ::std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    ::std::string token_1 = response.substr(0, del_pos);
    ::std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = ::std::atoi(token_1.c_str());
    val_2 = ::std::atoi(token_2.c_str());
  }
  
  void set_motor_values(int val_1, int val_2)
  {
    ::std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    ::std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

  // Buzzer kontrolü için yeni fonksiyon
  void set_buzzer_state(bool buzzer_on)
  {
    ::std::stringstream ss;
    ss << "b " << (buzzer_on ? "1" : "0") << "\r";
    send_msg(ss.str());
  }

  // DFPlayer ses komutları için yeni fonksiyonlar
  void play_sound_1()
  {
    ::std::stringstream ss;
    ss << "s 1\r";  // Arduino'ya "s 1" komutu gönder (ses 1'i çal)
    send_msg(ss.str());
  }

  void play_sound_2()
  {
    ::std::stringstream ss;
    ss << "s 2\r";  // Arduino'ya "s 2" komutu gönder (ses 2'yi çal)
    send_msg(ss.str());
  }

  // Genel ses çalma fonksiyonu (1-2 arası)
  void play_sound(int sound_number)
  {
    if (sound_number >= 1 && sound_number <= 2)
    {
      ::std::stringstream ss;
      ss << "s " << sound_number << "\r";
      send_msg(ss.str());
    }
  }

  // Servo motor kontrolü fonksiyonları
  void trigger_servo(int servo_index)
  {
    ::std::stringstream ss;
    ss << "v " << servo_index << "\r";  // Arduino'ya "v 0" komutu gönder (servo tetikle)
    ::std::string response = send_msg(ss.str());
    
    // Response'u kontrol et (optional)
    if (response.find("OK") == ::std::string::npos && 
        response.find("Invalid") != ::std::string::npos) {
      ::std::cerr << "Servo command failed: " << response << ::std::endl;
    }
  }

  // Ana servo tetikleme fonksiyonu (servo 0 için)
  void trigger_servo_movement()
  {
    trigger_servo(0);  // Servo 0'ı tetikle (90 derece git, 15 saniye bekle, geri dön)
  }

  // Servo durumu sorgulama (isteğe bağlı - gelecekte kullanılabilir)
  void read_servo_status(int servo_index)
  {
    ::std::stringstream ss;
    ss << "t " << servo_index << "\r";  // Arduino'ya servo read komutu
    ::std::string response = send_msg(ss.str());
    // Response işleme burada yapılabilir
  }

  // BATTERY MONITORING fonksiyonları
  void read_battery_info(float &voltage, float &percentage)
  {
    ::std::string response = send_msg("f\r");  // 'f' komutu gönder
    
    // Parse response format: "voltage:percentage" (örn: "12.45:85.3")
    ::std::string delimiter = ":";
    size_t del_pos = response.find(delimiter);
    
    if (del_pos != ::std::string::npos) {
      ::std::string voltage_str = response.substr(0, del_pos);
      ::std::string percentage_str = response.substr(del_pos + delimiter.length());
      
      try {
        voltage = ::std::stof(voltage_str);
        percentage = ::std::stof(percentage_str);
      } catch (const ::std::exception& e) {
        // Parse hatası durumunda varsayılan değerler
        voltage = 0.0f;
        percentage = 0.0f;
      }
    } else {
      // Delimiter bulunamadığında varsayılan değerler
      voltage = 0.0f;
      percentage = 0.0f;
    }
  }

  // Battery durumunu tek seferde okuma ve doğrulama
  bool get_battery_status(float &voltage, float &percentage)
  {
    try {
      read_battery_info(voltage, percentage);
      // Makul değerler kontrolü (12V sistem için)
      return (voltage > 8.0f && voltage < 16.0f && percentage >= 0.0f && percentage <= 100.0f);
    } catch (...) {
      voltage = 0.0f;
      percentage = 0.0f;
      return false;
    }
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP