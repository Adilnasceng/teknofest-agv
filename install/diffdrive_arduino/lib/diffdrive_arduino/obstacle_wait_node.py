#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import time
import math

class ObstacleWaitNode(Node):
    def __init__(self):
        super().__init__('obstacle_wait_node')
        
        # Parametreler
        self.declare_parameter('obstacle_distance_threshold', 0.8)  # metre
        self.declare_parameter('obstacle_angle_range', 60.0)        # derece (önde ±30°)
        self.declare_parameter('wait_duration', 15.0)               # saniye
        self.declare_parameter('min_scan_points', 5)                # minimum engel noktası sayısı
        self.declare_parameter('enable_obstacle_wait', True)        # özelliği aktif/pasif
        self.declare_parameter('ignore_duration', 10.0)             # bekleme sonrası ignore süresi
        self.declare_parameter('enable_buzzer', True)               # buzzer kontrolü aktif/pasif
        
        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.obstacle_angle_range = self.get_parameter('obstacle_angle_range').value
        self.wait_duration = self.get_parameter('wait_duration').value
        self.min_scan_points = self.get_parameter('min_scan_points').value
        self.enable_obstacle_wait = self.get_parameter('enable_obstacle_wait').value
        self.ignore_duration = self.get_parameter('ignore_duration').value
        self.enable_buzzer = self.get_parameter('enable_buzzer').value
        
        # QoS profili
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        self.cmd_vel_nav_subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',  # Navigation'dan gelen komutlar
            self.cmd_vel_nav_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_status_publisher = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.node_status_publisher = self.create_publisher(String, '/obstacle_wait_status', 10)
        
        # Buzzer kontrolü için publisher
        self.buzzer_control_publisher = self.create_publisher(Bool, '/buzzer_control', 10)
        
        # Durum değişkenleri
        self.obstacle_detected = False
        self.last_nav_cmd = Twist()
        self.buzzer_active = False
        
        # Ana state machine
        self.state = "NORMAL"  # NORMAL, WAITING, IGNORING
        self.state_start_time = None
        
        # Timer - periyodik kontrol
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Buzzer timer - buzzer pattern için
        self.buzzer_timer = self.create_timer(0.5, self.buzzer_pattern_callback)
        self.buzzer_pattern_state = False
        
        self.get_logger().info(f'🤖 Obstacle Wait Node başlatıldı')
        self.get_logger().info(f'📏 Engel mesafesi: {self.obstacle_distance_threshold}m')
        self.get_logger().info(f'📐 Engel açısı: ±{self.obstacle_angle_range/2}°')
        self.get_logger().info(f'⏰ Bekleme süresi: {self.wait_duration}s')
        self.get_logger().info(f'🚫 Ignore süresi: {self.ignore_duration}s')
        self.get_logger().info(f'🔊 Buzzer kontrolü: {"Aktif" if self.enable_buzzer else "Pasif"}')
        
        # Başlangıç durumu
        self.publish_status("INITIALIZED - Ready to detect obstacles")

    def scan_callback(self, msg):
        """LaserScan verilerini işle"""
        if not self.enable_obstacle_wait or self.state == "IGNORING":
            self.obstacle_detected = False
            return
            
        self.obstacle_detected = self.detect_front_obstacle(msg)
        
        # Obstacle status publish et
        obstacle_msg = Bool()
        obstacle_msg.data = self.obstacle_detected
        self.obstacle_status_publisher.publish(obstacle_msg)

    def detect_front_obstacle(self, scan_msg):
        """Robotun önündeki engeli tespit et"""
        if len(scan_msg.ranges) == 0:
            return False
        
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        # Önde bakılacak açı aralığını hesapla
        half_angle_range = math.radians(self.obstacle_angle_range / 2.0)
        
        obstacle_points = 0
        total_points_checked = 0
        
        for i, distance in enumerate(scan_msg.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue
                
            # Bu noktanın açısını hesapla
            angle = angle_min + i * angle_increment
            
            # Açıyı -π ile π arasına normalize et
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            # Sadece önde ±half_angle_range açısındaki noktaları kontrol et
            if abs(angle) <= half_angle_range:
                total_points_checked += 1
                if distance < self.obstacle_distance_threshold:
                    obstacle_points += 1
        
        # Yeterli engel noktası varsa True döndür
        return obstacle_points >= self.min_scan_points

    def cmd_vel_nav_callback(self, msg):
        """Navigation'dan gelen cmd_vel komutlarını yakala"""
        self.last_nav_cmd = msg

    def control_loop(self):
        """Ana state machine"""
        if not self.enable_obstacle_wait:
            # Özellik kapalıysa doğrudan navigation komutlarını geçir
            self.cmd_vel_publisher.publish(self.last_nav_cmd)
            self.state = "NORMAL"
            self.set_buzzer_state(False)
            return
        
        current_time = time.time()
        
        # State machine
        if self.state == "NORMAL":
            # Normal çalışma - engel kontrol et
            if self.obstacle_detected and self.is_robot_moving():
                self.start_waiting()
            else:
                # Normal hareket
                self.cmd_vel_publisher.publish(self.last_nav_cmd)
                self.set_buzzer_state(False)
                
        elif self.state == "WAITING":
            # Bekleme durumu
            elapsed = current_time - self.state_start_time
            remaining = self.wait_duration - elapsed
            
            if not self.obstacle_detected:
                # Engel kalktı
                self.get_logger().info("✅ Engel kalktı! Harekete devam.")
                self.state = "NORMAL"
                self.state_start_time = None
                self.set_buzzer_state(False)
                self.publish_status("OBSTACLE_CLEARED - Resuming movement")
                
            elif elapsed >= self.wait_duration:
                # Bekleme süresi doldu - ignore moduna geç
                self.start_ignoring()
                
            else:
                # Beklemeye devam - robot durdur ve buzzer çal
                stop_cmd = Twist()
                self.cmd_vel_publisher.publish(stop_cmd)
                self.set_buzzer_state(True)  # Buzzer'ı aktif et
                
                # Log her 3 saniyede bir
                if int(remaining * 10) % 30 == 0:
                    self.get_logger().info(f"🛑 Bekleniyor... Kalan: {remaining:.1f}s 🔊")
                    self.publish_status(f"WAITING - remaining: {remaining:.1f}s")
                    
        elif self.state == "IGNORING":
            # Ignore durumu - navigation'ın çalışmasına izin ver
            elapsed = current_time - self.state_start_time
            remaining = self.ignore_duration - elapsed
            
            if elapsed >= self.ignore_duration:
                # Ignore süresi doldu - normale dön
                self.get_logger().info("🔄 Ignore süresi doldu. Normal moda dönülüyor.")
                self.state = "NORMAL"
                self.state_start_time = None
                self.set_buzzer_state(False)
                self.publish_status("IGNORE_ENDED - Normal operation resumed")
            else:
                # Navigation komutlarını geçir ve buzzer'ı kapat
                self.cmd_vel_publisher.publish(self.last_nav_cmd)
                self.set_buzzer_state(False)
                
                # Log her 3 saniyede bir
                if int(remaining * 10) % 30 == 0:
                    self.get_logger().info(f"🚫 Ignore modu... Kalan: {remaining:.1f}s")
                    self.publish_status(f"IGNORING - remaining: {remaining:.1f}s")

    def start_waiting(self):
        """Bekleme durumunu başlat"""
        self.state = "WAITING"
        self.state_start_time = time.time()
        self.get_logger().info(f"🛑 ENGEL TESPİT EDİLDİ! {self.wait_duration} saniye bekleniyor... 🔊")
        self.publish_status(f"OBSTACLE_DETECTED - starting {self.wait_duration}s wait")
        self.set_buzzer_state(True)

    def start_ignoring(self):
        """Ignore durumunu başlat"""
        self.state = "IGNORING"
        self.state_start_time = time.time()
        self.get_logger().info(f"⏰ Bekleme süresi doldu! {self.ignore_duration} saniye ignore modu...")
        self.get_logger().info("🔄 Navigation yeni rota planlayabilir.")
        self.publish_status(f"WAIT_TIMEOUT - starting {self.ignore_duration}s ignore period")
        self.set_buzzer_state(False)

    def is_robot_moving(self):
        """Robot hareket ediyor mu kontrol et"""
        cmd = self.last_nav_cmd
        return (abs(cmd.linear.x) > 0.01 or 
                abs(cmd.linear.y) > 0.01 or 
                abs(cmd.angular.z) > 0.01)

    def publish_status(self, status):
        """Durum mesajı yayınla"""
        status_msg = String()
        status_msg.data = status
        self.node_status_publisher.publish(status_msg)

    def set_buzzer_state(self, active):
        """Buzzer durumunu ayarla"""
        if not self.enable_buzzer:
            return
            
        if self.buzzer_active != active:
            self.buzzer_active = active
            buzzer_msg = Bool()
            buzzer_msg.data = active
            self.buzzer_control_publisher.publish(buzzer_msg)
            
            if active:
                self.get_logger().info("🔊 Buzzer ON - Obstacle detected!")
            else:
                self.get_logger().info("🔇 Buzzer OFF")

    def buzzer_pattern_callback(self):
        """Buzzer pattern - intermittent beeping during obstacle wait"""
        if self.state == "WAITING" and self.buzzer_active and self.enable_buzzer:
            # Toggle pattern: ON-OFF-ON-OFF every 0.5 seconds
            pattern_msg = Bool()
            pattern_msg.data = self.buzzer_pattern_state
            self.buzzer_control_publisher.publish(pattern_msg)
            self.buzzer_pattern_state = not self.buzzer_pattern_state

    def set_enable_obstacle_wait(self, enable):
        """Engel bekleme özelliğini aktif/pasif yap"""
        self.enable_obstacle_wait = enable
        if not enable:
            self.state = "NORMAL"
            self.state_start_time = None
            self.set_buzzer_state(False)
        
        status = "ENABLED" if enable else "DISABLED"
        self.get_logger().info(f'Obstacle wait özelliği: {status}')
        self.publish_status(f"FEATURE_{status}")

    def set_enable_buzzer(self, enable):
        """Buzzer özelliğini aktif/pasif yap"""
        self.enable_buzzer = enable
        if not enable:
            self.set_buzzer_state(False)
        
        status = "ENABLED" if enable else "DISABLED"
        self.get_logger().info(f'Buzzer kontrolü: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleWaitNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node durduruldu')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()