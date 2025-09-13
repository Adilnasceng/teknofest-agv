#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial
import json
import time
from std_msgs.msg import String

class RFIDNode(Node):
    def __init__(self):
        super().__init__('rfid_reader_node')
        
        # Parameters tanımla
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        
        # Parameters'ı al
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Publishers oluştur
        self.rfid_pub = self.create_publisher(String, '/rfid_data', 10)
        self.status_pub = self.create_publisher(String, '/rfid_status', 10)
        
        # Serial bağlantı
        self.ser = None
        
        # Timer - serial port'u sürekli kontrol etmek için
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # İlk bağlantıyı kur
        self.connect_serial()
        
        self.get_logger().info("RFID Node başlatıldı (ROS2 Humble)")
        
    def connect_serial(self):
        """Serial bağlantısını kur"""
        if self.ser is not None:
            return
            
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Arduino'nun reset olması için bekle
            self.get_logger().info(f"Serial port {self.serial_port} bağlandı")
            
            # Status mesajı yayınla
            status_msg = String()
            status_msg.data = json.dumps({
                "status": "connected",
                "port": self.serial_port,
                "baud_rate": self.baud_rate,
                "timestamp": time.time()
            })
            self.status_pub.publish(status_msg)
            
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port açılamadı: {e}")
            self.ser = None
    
    def parse_arduino_data(self, line):
        """Arduino'dan gelen veriyi parse et"""
        line = line.strip()
        
        if line == "ARDUINO_READY":
            self.get_logger().info("Arduino hazır")
            status_msg = String()
            status_msg.data = json.dumps({
                "status": "arduino_ready",
                "timestamp": time.time()
            })
            self.status_pub.publish(status_msg)
            
        elif line.startswith("RFID_DATA:"):
            # JSON verisini çıkar
            json_str = line.replace("RFID_DATA:", "")
            try:
                data = json.loads(json_str)
                
                # Sadece Okunan ve Yazılan alanlarını yayınla
                msg = String()
                okunan = data.get("read_data", "")
                yazilan = data.get("write_data", "")
                msg.data = f"Okunan:{okunan} Yazılan:{yazilan}"
                
                # Veriyi yayınla
                self.rfid_pub.publish(msg)
                self.get_logger().info(f"RFID verisi yayınlandı -> {msg.data}")
                
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON parse hatası: {e}")
                
        elif line.startswith("ERROR_"):
            # Hata mesajları
            error_type = line.split(":")[0]
            error_msg = line.split(":", 1)[1] if ":" in line else "Bilinmeyen hata"
            
            self.get_logger().warning(f"Arduino hatası - {error_type}: {error_msg}")
            
            # Hata mesajını da yayınla
            error_data = {
                "node": "rfid_reader",
                "success": False,
                "error_type": error_type,
                "error_message": error_msg,
                "ros_timestamp": time.time(),
                "ros_time_nanosec": self.get_clock().now().nanoseconds
            }
            
            msg = String()
            msg.data = json.dumps(error_data)
            self.rfid_pub.publish(msg)
    
    def timer_callback(self):
        """Timer callback - serial port'u sürekli kontrol et"""
        if self.ser is None:
            # Bağlantıyı yeniden dene
            self.connect_serial()
            return
            
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_arduino_data(line)
                    
        except serial.SerialException as e:
            self.get_logger().error(f"Serial okuma hatası: {e}")
            self.ser.close()
            self.ser = None
            
        except UnicodeDecodeError as e:
            self.get_logger().warning(f"Unicode decode hatası: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Beklenmeyen hata: {e}")
    
    def destroy_node(self):
        """Node kapatılırken temizlik"""
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    rfid_node = RFIDNode()
    
    try:
        rclpy.spin(rfid_node)
    except KeyboardInterrupt:
        pass
    finally:
        rfid_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
