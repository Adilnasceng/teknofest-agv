#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import time

class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_node')
        
        # Parametreler
        self.declare_parameter('test_interval', 30.0)
        self.declare_parameter('enable_auto_test', False)
        self.declare_parameter('debug_mode', True)
        
        self.test_interval = self.get_parameter('test_interval').value
        self.enable_auto_test = self.get_parameter('enable_auto_test').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Service client - hardware interface ile konuşmak için
        self.servo_service_client = self.create_client(Trigger, '/trigger_servo')
        
        # Subscriber - servo durumunu dinle
        self.servo_status_subscription = self.create_subscription(
            Bool,
            '/servo_status',
            self.servo_status_callback,
            10
        )
        
        # Service'in hazır olmasını bekle
        self.service_ready = False
        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)
        
        # Otomatik test timer (eğer aktifse)
        if self.enable_auto_test:
            self.auto_test_timer = self.create_timer(self.test_interval, self.auto_trigger_servo)
            self.get_logger().info(f'⚡ Otomatik test aktif - {self.test_interval} saniyede bir servo tetiklenecek')
        
        # Manuel komut için input handling
        self.input_timer = self.create_timer(0.1, self.check_manual_input)
        self.last_trigger_time = 0
        
        self.get_logger().info('🔧 Servo Test Node başlatıldı')
        self.get_logger().info('📞 Servis: /trigger_servo')
        self.get_logger().info('📡 Status topic: /servo_status')
        self.get_logger().info('💡 Manuel test için: ros2 service call /trigger_servo std_srvs/srv/Trigger')
        
        if not self.enable_auto_test:
            self.get_logger().info('⌨️ Otomatik test kapalı - Manuel çağrı bekleniyor')

    def check_service_availability(self):
        """Hardware service'in hazır olup olmadığını kontrol et"""
        if self.servo_service_client.service_is_ready():
            if not self.service_ready:
                self.service_ready = True
                self.get_logger().info('✅ Hardware servo service bağlantısı kuruldu!')
        else:
            if self.service_ready:
                self.service_ready = False
                self.get_logger().warn('⚠️ Hardware servo service bağlantısı kesildi!')

    def servo_status_callback(self, msg):
        """Servo status mesajını al"""
        if msg.data:
            self.get_logger().info('📡 Servo status alındı: TETIKLENDI ✅')
        else:
            self.get_logger().info('📡 Servo status alındı: Normal 🔧')

    def auto_trigger_servo(self):
        """Otomatik servo tetikleme"""
        if self.service_ready:
            self.trigger_servo_service("Otomatik Test")
        else:
            self.get_logger().warn('🚫 Otomatik test - Servo servisi hazır değil!')

    def check_manual_input(self):
        """Manuel input kontrolü - Bu fonksiyon expansion için hazır"""
        # Gelecekte manuel klavye input'u için kullanılabilir
        pass

    def trigger_servo_service(self, trigger_source="Manuel"):
        """Servo service'ini çağır"""
        current_time = time.time()
        
        # Debounce: Son tetiklemeden en az 2 saniye geçmiş olmalı
        if current_time - self.last_trigger_time < 2.0:
            self.get_logger().warn('⚠️ Çok hızlı tetikleme! En az 2 saniye bekleyin.')
            return
            
        try:
            self.last_trigger_time = current_time
            
            request = Trigger.Request()
            
            # Async call yaparak blocking'i önle
            future = self.servo_service_client.call_async(request)
            future.add_done_callback(
                lambda fut: self.handle_servo_service_response(fut, trigger_source)
            )
            
            self.get_logger().info(f'🔧 Servo tetikleme isteği gönderildi ({trigger_source})')
            
        except Exception as e:
            self.get_logger().error(f'Servo service call hatası: {e}')

    def handle_servo_service_response(self, future, trigger_source):
        """Service response'unu handle et"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Servo başarıyla tetiklendi! ({trigger_source})')
                self.get_logger().info(f'📝 Yanıt: {response.message}')
                self.get_logger().info('🔧 Servo şimdi 90 dereceye gidip 5 saniye bekleyecek!')
            else:
                self.get_logger().error(f'❌ Servo tetikleme başarısız ({trigger_source}): {response.message}')
                
        except Exception as e:
            self.get_logger().error(f'Servo service response hatası ({trigger_source}): {e}')

    def manual_trigger(self):
        """Programatik manuel tetikleme"""
        if self.service_ready:
            self.trigger_servo_service("Programatik Manuel")
        else:
            self.get_logger().warn('🚫 Manuel tetikleme - Servo servisi hazır değil!')

    def enable_auto_test_mode(self, enable, interval=30.0):
        """Otomatik test modunu dinamik olarak aktif/pasif et"""
        if enable and not self.enable_auto_test:
            self.enable_auto_test = True
            self.test_interval = interval
            self.auto_test_timer = self.create_timer(self.test_interval, self.auto_trigger_servo)
            self.get_logger().info(f'⚡ Otomatik test aktif edildi - {interval} saniye aralık')
        elif not enable and self.enable_auto_test:
            self.enable_auto_test = False
            if hasattr(self, 'auto_test_timer'):
                self.auto_test_timer.cancel()
            self.get_logger().info('⏸️ Otomatik test pasif edildi')


def main(args=None):
    rclpy.init(args=args)
    node = ServoTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Servo Test Node durduruldu')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()