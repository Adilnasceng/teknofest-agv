#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Abonelikler ve Yayıncılar
        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        # Hata ayıklama için işlenmiş görüntüyü yayınlayacak publisher
        self.processed_image_pub = self.create_publisher(Image, '/processed_image', 10)
        
        # Çizgi takibi durumu publisher'ı
        self.line_status_pub = self.create_publisher(Bool, '/line_follower_status', 10)
        
        # Service server - çizgi takibini başlat/durdur
        self.control_service = self.create_service(
            SetBool,
            'line_follower_control',
            self.control_callback
        )

        self.bridge = CvBridge()
        
        # Kontrolcü parametreleri
        self.forward_speed = 0.1
        self.kp = 0.005
        
        # Çizgi takibi durumu
        self.is_following = False
        self.line_detected = False
        
        # Timer - durum yayını için
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('🔍 Çizgi Takip Düğümü Başlatıldı (Servis Kontrollü).')
        self.get_logger().info('📞 Servis: /line_follower_control')

    def control_callback(self, request, response):
        """Çizgi takibini başlat/durdur"""
        if request.data:
            self.is_following = True
            self.get_logger().info('🟢 Çizgi takibi BAŞLATILDI')
            response.success = True
            response.message = "Çizgi takibi başlatıldı"
        else:
            self.is_following = False
            # Robot durdur
            stop_msg = Twist()
            self.velocity_publisher.publish(stop_msg)
            self.get_logger().info('🔴 Çizgi takibi DURDURULDU')
            response.success = True
            response.message = "Çizgi takibi durduruldu"
        
        return response

    def publish_status(self):
        """Çizgi takibi durumunu yayınla"""
        status_msg = Bool()
        status_msg.data = self.is_following and self.line_detected
        self.line_status_pub.publish(status_msg)

    def image_callback(self, msg):
        # Çizgi takibi aktif değilse görüntü işleme yapma
        if not self.is_following:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge hatası: {e}')
            return

        # Görüntü işleme
        height, width, _ = cv_image.shape
        roi_height_start = int(height * 0.7)
        roi = cv_image[roi_height_start:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

        M = cv2.moments(thresh)
        twist_msg = Twist()

        if M['m00'] > 0:
            self.line_detected = True
            cx = int(M['m10'] / M['m00'])
            
            # Kontrol Mantığı
            roi_width = roi.shape[1]
            error = (roi_width // 2) - cx
            
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = self.kp * float(error)
            
            if self.is_following:  # Sadece takip modundayken log
                self.get_logger().info(f'🔍 Çizgi takip ediliyor. Hata: {error}, Dönüş: {twist_msg.angular.z:.2f}', throttle_duration_sec=1.0)
        else:
            self.line_detected = False
            if self.is_following:
                self.get_logger().warn('⚠️ Çizgi bulunamadı!', throttle_duration_sec=2.0)
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        # Sadece takip modundayken hareket komutları gönder
        if self.is_following:
            self.velocity_publisher.publish(twist_msg)

        # İşlenmiş görüntüyü yayınla
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(thresh, 'mono8')
            self.processed_image_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'İşlenmiş görüntü yayınlanırken hata: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()