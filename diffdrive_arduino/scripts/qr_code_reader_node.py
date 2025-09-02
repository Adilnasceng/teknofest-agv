#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time
from datetime import datetime

class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_reader')
        
        # Publisher oluştur
        self.publisher_ = self.create_publisher(String, 'qr_code_data', 10)
        
        # Parametreler
        self.declare_parameter('serial_port', '/dev/qr_reader')  # Robot /dev/ttyACM0 kullanıyor, QR için farklı port
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('timeout', 1.0)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        self.serial_connection = None
        self.running = True
        self.last_qr_data = ""  # Aynı QR'ı tekrar okumamak için
        
        # Serial bağlantısını kur
        self.setup_serial_connection()
        
        # Okuma thread'ini başlat
        self.read_thread = threading.Thread(target=self.read_qr_data)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        self.get_logger().info('QR Code Reader node başlatıldı')
        self.get_logger().info(f'Serial port: {self.serial_port}, Baud rate: {self.baud_rate}')
        
    def setup_serial_connection(self):
        """Serial bağlantısını kurmaya çalış"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            self.get_logger().info(f'Serial bağlantı kuruldu: {self.serial_port}')
            time.sleep(2)  # Bağlantının stabilize olması için bekle
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial bağlantı hatası: {e}')
            self.get_logger().warn('Serial bağlantı kurulamadı, tekrar denenecek...')
            
    def read_qr_data(self):
        """QR code verilerini oku"""
        reconnect_attempts = 0
        max_reconnect_attempts = 5
        
        while self.running:
            try:
                if self.serial_connection is None or not self.serial_connection.is_open:
                    if reconnect_attempts < max_reconnect_attempts:
                        self.get_logger().warn(f'Serial bağlantı yeniden deneniyor... ({reconnect_attempts + 1}/{max_reconnect_attempts})')
                        self.setup_serial_connection()
                        reconnect_attempts += 1
                        time.sleep(3)
                        continue
                    else:
                        self.get_logger().error('Maximum yeniden bağlanma denemesi aşıldı')
                        time.sleep(10)
                        reconnect_attempts = 0
                        continue
                
                # Seri porttan veri oku
                if self.serial_connection.in_waiting > 0:
                    try:
                        # Satır bazında oku
                        line = self.serial_connection.readline().decode('utf-8').strip()
                        
                        if line:
                            # QR code verisini temizle
                            qr_data = line.replace('\r', '').replace('\n', '').strip()
                            
                            if qr_data and qr_data != self.last_qr_data:
                                # Aynı QR'ı tekrar okumamak için kontrol
                                self.last_qr_data = qr_data
                                
                                # Mesaj oluştur ve yayınla
                                msg = String()
                                msg.data = qr_data
                                
                                self.publisher_.publish(msg)
                                
                                # Log mesajı
                                timestamp = datetime.now().strftime("%H:%M:%S")
                                self.get_logger().info(f'[{timestamp}] QR Code okundu: "{qr_data}"')
                                
                                reconnect_attempts = 0  # Başarılı okuma sonrası reset
                                
                    except UnicodeDecodeError as e:
                        self.get_logger().warn(f'Unicode decode hatası: {e}')
                        
                else:
                    time.sleep(0.1)  # CPU kullanımını azalt
                    
            except serial.SerialException as e:
                self.get_logger().error(f'Serial okuma hatası: {e}')
                if self.serial_connection:
                    try:
                        self.serial_connection.close()
                    except:
                        pass
                self.serial_connection = None
                time.sleep(1)
                
            except Exception as e:
                self.get_logger().error(f'Beklenmeyen hata: {e}')
                time.sleep(1)
                
    def destroy_node(self):
        """Node kapanırken temizlik yap"""
        self.running = False
        
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.close()
                self.get_logger().info('Serial bağlantı kapatıldı')
            except:
                pass
                
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    qr_reader = QRCodeReader()
    
    try:
        rclpy.spin(qr_reader)
    except KeyboardInterrupt:
        pass
    finally:
        qr_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()