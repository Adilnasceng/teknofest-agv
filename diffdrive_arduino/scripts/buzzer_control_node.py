#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import time

class BuzzerControlNode(Node):
    def __init__(self):
        super().__init__('buzzer_control_node')
        
        # Subscriber - buzzer control topic'ini dinle
        self.buzzer_subscription = self.create_subscription(
            Bool,
            '/buzzer_control',
            self.buzzer_callback,
            10
        )
        
        # Publisher - buzzer durumunu paylaş
        self.buzzer_status_publisher = self.create_publisher(Bool, '/buzzer_status', 10)
        
        # Service client - hardware interface ile konuşmak için
        self.buzzer_service_client = self.create_client(SetBool, '/set_buzzer_state')
        
        # Service'in hazır olmasını bekle
        self.service_ready = False
        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)
        
        # Son buzzer durumu
        self.last_buzzer_state = False
        
        self.get_logger().info('🔊 Buzzer Control Node başlatıldı')

    def check_service_availability(self):
        """Hardware service'in hazır olup olmadığını kontrol et"""
        if self.buzzer_service_client.service_is_ready():
            if not self.service_ready:
                self.service_ready = True
                self.get_logger().info('✅ Hardware buzzer service bağlantısı kuruldu!')
        else:
            if self.service_ready:
                self.service_ready = False
                self.get_logger().warn('⚠️ Hardware buzzer service bağlantısı kesildi!')

    def buzzer_callback(self, msg):
        """Buzzer control mesajını al ve hardware'a ilet"""
        buzzer_on = msg.data
        
        if self.last_buzzer_state != buzzer_on:
            self.last_buzzer_state = buzzer_on
            
            if self.service_ready:
                self.call_hardware_buzzer_service(buzzer_on)
            else:
                self.get_logger().warn('🚫 Hardware service hazır değil, buzzer komutu gönderilemedi')

    def call_hardware_buzzer_service(self, buzzer_on):
        """Hardware buzzer service'ini çağır"""
        try:
            request = SetBool.Request()
            request.data = buzzer_on
            
            # Async call yaparak blocking'i önle
            future = self.buzzer_service_client.call_async(request)
            future.add_done_callback(
                lambda fut: self.handle_service_response(fut, buzzer_on)
            )
            
        except Exception as e:
            self.get_logger().error(f'Buzzer service call hatası: {e}')

    def handle_service_response(self, future, buzzer_on):
        """Service response'unu handle et"""
        try:
            response = future.result()
            if response.success:
                # Status publish et
                status_msg = Bool()
                status_msg.data = buzzer_on
                self.buzzer_status_publisher.publish(status_msg)
                
                self.get_logger().info(f'🔊 Buzzer {"ON" if buzzer_on else "OFF"} - {response.message}')
            else:
                self.get_logger().error(f'Buzzer service başarısız: {response.message}')
                
        except Exception as e:
            self.get_logger().error(f'Service response hatası: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Buzzer Control Node durduruldu')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()