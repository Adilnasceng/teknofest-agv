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
            '/line_camera/image_raw',
            self.image_callback,
            10)
        
        # Twist Mux için özel topic kullan
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_line_follow',  # Twist mux bu topic'i dinleyecek
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
        
        # Görüntü işleme parametreleri
        self.declare_parameter('adaptive_threshold', True)
        self.declare_parameter('threshold_value', 80)  # Daha yüksek değer
        self.declare_parameter('adaptive_block_size', 15)  # Adaptive threshold için
        self.declare_parameter('adaptive_c', 8)  # Adaptive threshold sabiti
        self.declare_parameter('morphology_enabled', True)
        self.declare_parameter('min_line_area', 500)  # Minimum çizgi alanı
        self.declare_parameter('roi_height_ratio', 0.6)  # ROI yükseklik oranı
        
        # Çizgi takibi durumu
        self.is_following = False
        self.line_detected = False
        
        # Timer - durum yayını için
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        # Heartbeat timer - Twist mux timeout için
        self.heartbeat_timer = self.create_timer(0.2, self.heartbeat_callback)
        
        # Log kontrol için
        self.last_log_time = 0
        self.log_interval = 3.0  # 3 saniyede bir log
        
        # Histogram analizi için
        self.brightness_history = []
        self.max_history_length = 10
        
        self.get_logger().info('🔍 Çizgi Takip Düğümü Başlatıldı (İyileştirilmiş Algılama).')
        self.get_logger().info('📞 Servis: /line_follower_control')
        self.get_logger().info('📡 Twist Mux Topic: /cmd_vel_line_follow')
        self.get_logger().info('⏸️ Çizgi takibi DEVRE DIŞI - Servis çağrısı bekleniyor')

    def control_callback(self, request, response):
        """Çizgi takibini başlat/durdur"""
        if request.data:
            self.is_following = True
            self.get_logger().info('🟢 Çizgi takibi BAŞLATILDI (İyileştirilmiş Algılama)')
            response.success = True
            response.message = "Çizgi takibi başlatıldı - İyileştirilmiş algılama aktif"
        else:
            self.is_following = False
            # Robot durdur - Bu, twist mux üzerinden önceliği düşürür
            stop_msg = Twist()
            self.velocity_publisher.publish(stop_msg)
            self.get_logger().info('🔴 Çizgi takibi DURDURULDU (Navigation restored)')
            response.success = True
            response.message = "Çizgi takibi durduruldu - Navigation restored"
        
        return response

    def publish_status(self):
        """Çizgi takibi durumunu yayınla"""
        status_msg = Bool()
        status_msg.data = self.is_following and self.line_detected
        self.line_status_pub.publish(status_msg)

    def heartbeat_callback(self):
        """Twist mux timeout'unu önlemek için heartbeat gönder"""
        if self.is_following:
            # Aktif çizgi takibi sırasında sürekli komut gönder
            # Eğer görüntü işleme yapılmıyorsa durdurma komutu gönder
            if not hasattr(self, '_last_image_time') or \
               (self.get_clock().now().nanoseconds / 1e9 - self._last_image_time) > 1.0:
                # Görüntü gelmiyorsa durdur
                stop_msg = Twist()
                self.velocity_publisher.publish(stop_msg)

    def get_adaptive_threshold(self, gray_image):
        """Görüntü parlaklığına göre adaptif threshold değeri hesapla"""
        # Ortalama parlaklığı hesapla
        mean_brightness = np.mean(gray_image)
        
        # Parlaklık geçmişini güncelle
        self.brightness_history.append(mean_brightness)
        if len(self.brightness_history) > self.max_history_length:
            self.brightness_history.pop(0)
        
        # Ortalama parlaklık geçmişi
        avg_brightness = np.mean(self.brightness_history)
        
        # Adaptif threshold değeri
        if avg_brightness > 180:  # Çok parlak ortam
            return 200
        elif avg_brightness > 120:  # Orta parlak
            return 160
        elif avg_brightness > 80:   # Orta karanlık
            return 120
        else:  # Karanlık ortam
            return 100

    def process_image_for_line(self, cv_image):
        """Gelişmiş çizgi algılama işlemi"""
        height, width, _ = cv_image.shape
        
        # ROI (Region of Interest) belirleme
        roi_height_ratio = self.get_parameter('roi_height_ratio').value
        roi_height_start = int(height * roi_height_ratio)
        roi = cv_image[roi_height_start:, :]
        
        # Gri tonlamaya çevir
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Görüntüyü bulanıklaştır (gürültüyü azalt)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Adaptif veya sabit threshold
        if self.get_parameter('adaptive_threshold').value:
            # Adaptif threshold kullan
            block_size = self.get_parameter('adaptive_block_size').value
            c_value = self.get_parameter('adaptive_c').value
            
            # Block size tek sayı olmalı ve en az 3 olmalı
            if block_size % 2 == 0:
                block_size += 1
            block_size = max(3, block_size)
            
            thresh = cv2.adaptiveThreshold(
                blurred, 
                255, 
                cv2.ADAPTIVE_THRESH_MEAN_C, 
                cv2.THRESH_BINARY_INV, 
                block_size, 
                c_value
            )
        else:
            # Sabit threshold ama adaptif değer ile
            threshold_value = self.get_adaptive_threshold(blurred)
            _, thresh = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY_INV)
        
        # Morfolojik işlemler (gürültüyü temizle)
        if self.get_parameter('morphology_enabled').value:
            # Küçük gürültüleri temizle
            kernel_small = np.ones((3, 3), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel_small)
            
            # Çizgileri güçlendir
            kernel_large = np.ones((3, 7), np.uint8)  # Yatay çizgiler için
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_large)
        
        return thresh, roi

    def find_line_center(self, thresh_image):
        """Çizginin merkezini bul"""
        # Konturları bul
        contours, _ = cv2.findContours(thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0
        
        # En büyük konturu bul (ana çizgi olması muhtemel)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Minimum alan kontrolü
        min_area = self.get_parameter('min_line_area').value
        if area < min_area:
            return None, area
        
        # Moment hesaplama yöntemi
        M = cv2.moments(largest_contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            return cx, area
        
        return None, area

    def image_callback(self, msg):
        # Son görüntü zamanını kaydet
        self._last_image_time = self.get_clock().now().nanoseconds / 1e9
        
        # ÖNEMLİ: Çizgi takibi aktif değilse HİÇBİR İŞLEM YAPMA
        if not self.is_following:
            self.line_detected = False
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge hatası: {e}')
            return

        # Gelişmiş görüntü işleme
        thresh, roi = self.process_image_for_line(cv_image)
        
        # Çizgi merkezini bul
        line_center, line_area = self.find_line_center(thresh)
        
        twist_msg = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if line_center is not None:
            self.line_detected = True
            
            # Kontrol Mantığı
            roi_width = roi.shape[1]
            error = (roi_width // 2) - line_center
            
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = self.kp * float(error)
            
            # Sadece belirli aralıklarla log ver (spam'i önlemek için)
            if current_time - self.last_log_time >= self.log_interval:
                self.get_logger().info(
                    f'🔍 Çizgi takip ediliyor. Merkez: {line_center}, Alan: {line_area:.0f}, '
                    f'Hata: {error}, Dönüş: {twist_msg.angular.z:.2f}'
                )
                self.last_log_time = current_time
        else:
            self.line_detected = False
            # Çizgi bulunamadığında da belirli aralıklarla log ver
            if current_time - self.last_log_time >= self.log_interval:
                threshold_info = f"Threshold: {self.get_parameter('threshold_value').value}" if not self.get_parameter('adaptive_threshold').value else "Adaptive threshold"
                self.get_logger().warn(f'⚠️ Çizgi bulunamadı! Alan: {line_area:.0f}, {threshold_info}')
                self.get_logger().info('💡 Eğer çok hassasıysa: ros2 param set /line_follower_node threshold_value 220')
                self.last_log_time = current_time
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        # Hareket komutlarını gönder (sadece takip modunda)
        # Twist Mux bu komutu önceliğe göre işleyecek
        self.velocity_publisher.publish(twist_msg)

        # İşlenmiş görüntüyü yayınla (sadece takip modunda)
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(thresh, 'mono8')
            self.processed_image_pub.publish(processed_msg)
        except Exception as e:
            if current_time - self.last_log_time >= self.log_interval:
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