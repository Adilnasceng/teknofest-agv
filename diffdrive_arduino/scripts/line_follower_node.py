#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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

        self.bridge = CvBridge()
        
        # Kontrolcü parametreleri
        self.forward_speed = 0.1
        self.kp = 0.005
        
        self.get_logger().info('Çizgi Takip Düğümü Başlatıldı.')

    def image_callback(self, msg):
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
            cx = int(M['m10'] / M['m00'])
            
            # --- Görsel Hata Ayıklama için Merkez Çizimi ---
            # Bu çizim, yayınlanacak görüntüde görünecek
            # thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR) # Eğer renkli isterseniz
            # cv2.circle(thresh_color, (cx, int(roi.shape[0]/2)), 5, (0, 255, 0), -1)

            # Kontrol Mantığı
            roi_width = roi.shape[1]
            error = (roi_width // 2) - cx
            
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = self.kp * float(error)
            
            self.get_logger().info(f'Çizgi bulundu. Hata: {error}, Dönüş Hızı: {twist_msg.angular.z:.2f}', throttle_duration_sec=0.5)
        else:
            self.get_logger().warn('Çizgi bulunamadı!', throttle_duration_sec=1.0)
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        self.velocity_publisher.publish(twist_msg)

        # --- ARTIK GÖRSEL PENCERE YOK ---
        # cv2.imshow("İşlenmiş Görüntü (ROI)", thresh)
        # cv2.imshow("Orijinal Görüntü (ROI)", roi)
        # cv2.waitKey(1)
        
        # Bunun yerine, işlenmiş görüntüyü ROS topic olarak yayınla
        try:
            # Görüntüyü yayınlamak için tekrar ROS formatına çevir
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