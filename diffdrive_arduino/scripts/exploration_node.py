#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, Int32
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatusArray
import numpy as np
import random
import math
import time
from threading import Lock

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        
        # Parametreler
        self.declare_parameter('enable_exploration', True)
        self.declare_parameter('exploration_radius', 3.0)
        self.declare_parameter('min_frontier_size', 0.4)
        self.declare_parameter('goal_timeout', 35.0)
        self.declare_parameter('random_walk_probability', 0.1)
        self.declare_parameter('safe_distance', 0.9)
        self.declare_parameter('exploration_speed', 0.25)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('frontier_detection_threshold', 0.1)
        self.declare_parameter('return_to_start', True)          # Başlangıca dön
        self.declare_parameter('start_position_tolerance', 0.5)  # Başlangıç pozisyon toleransı
        
        # Parametreleri al
        self.enable_exploration = self.get_parameter('enable_exploration').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.random_walk_probability = self.get_parameter('random_walk_probability').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.exploration_speed = self.get_parameter('exploration_speed').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.frontier_threshold = self.get_parameter('frontier_detection_threshold').value
        self.return_to_start = self.get_parameter('return_to_start').value
        self.start_position_tolerance = self.get_parameter('start_position_tolerance').value
        
        # Thread safety
        self.lock = Lock()
        
        # QoS profilleri
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/diffbot_base_controller/odom',  # Sizin sisteminizdeki odom topic'i
            self.odom_callback,
            odom_qos
        )
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            odom_qos
        )
        
        # Navigation status
        self.nav_status_subscription = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav_status_callback,
            10
        )
        
        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_publisher = self.create_publisher(String, '/exploration_status', 10)
        self.frontier_publisher = self.create_publisher(Point, '/exploration_frontier', 10)
        self.debug_publisher = self.create_publisher(String, '/exploration_debug', 10)
        
        # Services
        self.enable_service = self.create_service(
            SetBool,
            'enable_exploration',
            self.enable_exploration_callback
        )
        
        # Durum değişkenleri
        self.current_map = None
        self.robot_position = None
        self.robot_orientation = None
        self.last_scan = None
        
        # Başlangıç pozisyonu
        self.start_position = None
        self.start_orientation = None
        self.start_position_recorded = False
        
        # Exploration state
        self.exploration_state = "WAITING"  # WAITING, EXPLORING, MOVING_TO_GOAL, STUCK, COMPLETED, RETURNING_HOME, FINISHED
        self.current_goal = None
        self.goal_sent_time = None
        self.stuck_count = 0
        self.max_stuck_count = 3
        self.explored_goals = []
        self.returning_home = False
        
        # Frontier detection
        self.frontiers = []
        self.last_frontier_update = 0
        self.frontier_update_interval = 3.0  # 3 saniyede bir frontier güncelle
        
        # Statistics
        self.total_goals_sent = 0
        self.successful_goals = 0
        self.exploration_start_time = None
        
        # Timer
        self.main_timer = self.create_timer(2.0, self.exploration_loop)
        self.frontier_timer = self.create_timer(3.0, self.update_frontiers)
        
        self.get_logger().info('🗺️ Exploration Node başlatıldı')
        self.get_logger().info(f'📏 Keşif yarıçapı: {self.exploration_radius}m')
        self.get_logger().info(f'🎯 Hedef timeout: {self.goal_timeout}s')
        self.get_logger().info(f'🔍 Exploration durumu: {"AKTIF" if self.enable_exploration else "PASIF"}')
        
        self.publish_status("INITIALIZED - Ready for exploration")

    def map_callback(self, msg):
        """Harita güncellemelerini al"""
        with self.lock:
            self.current_map = msg
            self.map_resolution = msg.info.resolution

    def odom_callback(self, msg):
        """Robot pozisyonunu güncelle"""
        with self.lock:
            self.robot_position = msg.pose.pose.position
            self.robot_orientation = msg.pose.pose.orientation
            
            # İlk pozisyonu başlangıç pozisyonu olarak kaydet
            if not self.start_position_recorded:
                self.start_position = msg.pose.pose.position
                self.start_orientation = msg.pose.pose.orientation
                self.start_position_recorded = True
                self.get_logger().info(f"📍 Başlangıç pozisyonu kaydedildi: ({self.start_position.x:.2f}, {self.start_position.y:.2f})")

    def scan_callback(self, msg):
        """Laser scan verilerini al"""
        with self.lock:
            self.last_scan = msg

    def nav_status_callback(self, msg):
        """Navigation durumunu izle"""
        if not self.current_goal:
            return
            
        for status in msg.status_list:
            if status.status == 4:  # SUCCEEDED
                if self.returning_home:
                    self.get_logger().info("🏠 Başlangıç pozisyonuna başarıyla döndü!")
                    self.exploration_state = "FINISHED"
                    self.returning_home = False
                    self.publish_status("RETURNED_HOME - Exploration finished")
                else:
                    self.get_logger().info("✅ Hedef başarıyla ulaşıldı!")
                    self.exploration_state = "EXPLORING"
                    self.successful_goals += 1
                    self.stuck_count = 0
                    self.add_explored_goal(self.current_goal)
                
                self.current_goal = None
                
            elif status.status in [5, 6]:  # CANCELED, ABORTED
                if self.returning_home:
                    self.get_logger().warn("⚠️ Başlangıç pozisyonuna dönüş başarısız!")
                    self.exploration_state = "STUCK"
                else:
                    self.get_logger().warn("⚠️ Hedef iptal edildi veya başarısız!")
                    self.exploration_state = "STUCK"
                self.stuck_count += 1

    def exploration_loop(self):
        """Ana exploration döngüsü"""
        if not self.enable_exploration:
            return
            
        if not self.is_ready():
            return
            
        current_time = time.time()
        
        # State machine
        if self.exploration_state == "WAITING":
            self.start_exploration()
            
        elif self.exploration_state == "EXPLORING":
            # Yeni hedef bul ve gönder
            goal = self.find_next_goal()
            if goal:
                self.send_goal(goal)
                self.exploration_state = "MOVING_TO_GOAL"
            else:
                # Frontier bulunamadı - exploration tamamlandı
                if self.return_to_start and self.start_position:
                    self.get_logger().info("🎉 Keşif tamamlandı! Başlangıç pozisyonuna dönülüyor...")
                    self.return_to_start_position()
                else:
                    self.get_logger().info("🎉 Keşif tamamlandı!")
                    self.exploration_state = "FINISHED"
                    self.publish_status("EXPLORATION_COMPLETED")
                
        elif self.exploration_state == "MOVING_TO_GOAL":
            # Hedef timeout kontrolü
            if self.goal_sent_time and (current_time - self.goal_sent_time > self.goal_timeout):
                self.get_logger().warn("⏰ Hedef timeout! Yeni hedef aranıyor...")
                self.exploration_state = "STUCK"
                self.stuck_count += 1
                
        elif self.exploration_state == "STUCK":
            self.handle_stuck_situation()
            
        elif self.exploration_state == "RETURNING_HOME":
            # Başlangıç pozisyonuna dönüş timeout kontrolü
            if self.goal_sent_time and (current_time - self.goal_sent_time > self.goal_timeout * 1.5):
                self.get_logger().warn("⏰ Başlangıça dönüş timeout! Yeniden deneniyor...")
                self.return_to_start_position()
                
        elif self.exploration_state == "FINISHED":
            # Keşif ve dönüş tamamlandı - bekle
            if int(current_time) % 30 == 0:  # 30 saniyede bir kontrol et
                if self.is_at_start_position():
                    self.get_logger().info("✅ Robot başlangıç pozisyonunda bekliyor")
                else:
                    self.get_logger().info("🔄 Pozisyon sapması tespit edildi, yeniden keşif başlatılıyor")
                    self.exploration_state = "EXPLORING"
            
        elif self.exploration_state == "COMPLETED":
            # Eski state - artık FINISHED kullanıyoruz
            self.exploration_state = "FINISHED"

    def update_frontiers(self):
        """Frontier'ları güncelle"""
        if not self.current_map or not self.robot_position:
            return
            
        current_time = time.time()
        if current_time - self.last_frontier_update < self.frontier_update_interval:
            return
            
        self.last_frontier_update = current_time
        
        with self.lock:
            self.frontiers = self.detect_frontiers()
            
        # Debug bilgisi
        if self.frontiers:
            self.debug_publish(f"Found {len(self.frontiers)} frontiers")
            # En yakın frontier'ı publish et
            if self.frontiers:
                closest = min(self.frontiers, key=lambda f: self.distance_to_point(f))
                frontier_msg = Point()
                frontier_msg.x = closest[0]
                frontier_msg.y = closest[1]
                self.frontier_publisher.publish(frontier_msg)

    def detect_frontiers(self):
        """Haritada frontier'ları (keşfedilmemiş sınırları) tespit et"""
        if not self.current_map:
            return []
            
        frontiers = []
        map_data = np.array(self.current_map.data).reshape(
            (self.current_map.info.height, self.current_map.info.width)
        )
        
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        resolution = self.current_map.info.resolution
        
        # Grid'i tara (daha hızlı olmak için her 2. pikseli kontrol et)
        for y in range(2, map_data.shape[0] - 2, 2):
            for x in range(2, map_data.shape[1] - 2, 2):
                # Bu nokta boş alan mı?
                if map_data[y, x] == 0:  # Free space
                    # Çevresinde bilinmeyen alan var mı?
                    has_unknown = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if map_data[y + dy, x + dx] == -1:  # Unknown
                                has_unknown = True
                                break
                        if has_unknown:
                            break
                    
                    if has_unknown:
                        # Grid koordinatlarını dünya koordinatlarına çevir
                        world_x = origin_x + x * resolution
                        world_y = origin_y + y * resolution
                        
                        # Robot pozisyonuna mesafe kontrol et
                        distance = math.sqrt(
                            (world_x - self.robot_position.x) ** 2 + 
                            (world_y - self.robot_position.y) ** 2
                        )
                        
                        if distance <= self.exploration_radius and distance >= self.min_frontier_size:
                            # Güvenli mesafe kontrolü
                            if self.is_safe_position(world_x, world_y):
                                frontiers.append((world_x, world_y))
        
        return frontiers

    def is_safe_position(self, x, y):
        """Pozisyonun güvenli olup olmadığını kontrol et"""
        if not self.current_map:
            return False
            
        map_data = np.array(self.current_map.data).reshape(
            (self.current_map.info.height, self.current_map.info.width)
        )
        
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        resolution = self.current_map.info.resolution
        
        # Güvenli mesafe için gerekli grid hücre sayısı
        safe_cells = int(self.safe_distance / resolution)
        
        # Pozisyonu grid'e çevir
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # Çevresindeki hücreleri kontrol et
        for dy in range(-safe_cells, safe_cells + 1):
            for dx in range(-safe_cells, safe_cells + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                # Grid sınırları içinde mi?
                if (0 <= check_x < map_data.shape[1] and 
                    0 <= check_y < map_data.shape[0]):
                    
                    # Bu hücre engel mi?
                    if map_data[check_y, check_x] > 50:  # Occupied
                        return False
        
        return True

    def find_next_goal(self):
        """Bir sonraki keşif hedefini bul"""
        if not self.frontiers:
            return None
            
        # Zaten keşfedilen hedefleri filtrele
        available_frontiers = []
        for frontier in self.frontiers:
            if not self.is_goal_explored(frontier):
                available_frontiers.append(frontier)
        
        if not available_frontiers:
            return None
        
        # Random walk olasılığı
        if random.random() < self.random_walk_probability:
            return random.choice(available_frontiers)
        
        # En yakın frontier'ı seç
        if self.robot_position:
            closest_frontier = min(
                available_frontiers,
                key=lambda f: self.distance_to_point(f)
            )
            return closest_frontier
        
        return random.choice(available_frontiers) if available_frontiers else None

    def send_goal(self, goal_point):
        """Hedefe gitme komutu gönder"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.position.x = goal_point[0]
        goal_msg.pose.position.y = goal_point[1]
        goal_msg.pose.position.z = 0.0
        
        # Yönelimi robotun bulunduğu yöne doğru ayarla
        if self.robot_orientation:
            goal_msg.pose.orientation = self.robot_orientation
        else:
            goal_msg.pose.orientation.w = 1.0
        
        self.goal_publisher.publish(goal_msg)
        self.current_goal = goal_point
        self.goal_sent_time = time.time()
        self.total_goals_sent += 1
        
        if self.returning_home:
            self.get_logger().info(f"🏠 Başlangıç pozisyonuna dönülüyor: ({goal_point[0]:.2f}, {goal_point[1]:.2f})")
            self.publish_status(f"RETURNING_HOME - Target: ({goal_point[0]:.2f}, {goal_point[1]:.2f})")
        else:
            self.get_logger().info(f"🎯 Yeni hedef gönderildi: ({goal_point[0]:.2f}, {goal_point[1]:.2f})")
            self.publish_status(f"GOAL_SENT - Target: ({goal_point[0]:.2f}, {goal_point[1]:.2f})")

    def return_to_start_position(self):
        """Başlangıç pozisyonuna dön"""
        if not self.start_position:
            self.get_logger().warn("⚠️ Başlangıç pozisyonu kaydedilmemiş!")
            self.exploration_state = "FINISHED"
            return
            
        # Başlangıç pozisyonunu hedef olarak ayarla
        start_goal = (self.start_position.x, self.start_position.y)
        
        # Zaten başlangıç pozisyonunda mı?
        if self.is_at_start_position():
            self.get_logger().info("✅ Robot zaten başlangıç pozisyonunda!")
            self.exploration_state = "FINISHED"
            self.publish_status("ALREADY_AT_HOME")
            return
            
        self.returning_home = True
        self.exploration_state = "RETURNING_HOME"
        self.send_goal(start_goal)

    def is_at_start_position(self):
        """Robot başlangıç pozisyonunda mı?"""
        if not self.start_position or not self.robot_position:
            return False
            
        distance = math.sqrt(
            (self.robot_position.x - self.start_position.x) ** 2 + 
            (self.robot_position.y - self.start_position.y) ** 2
        )
        
        return distance <= self.start_position_tolerance

    def handle_stuck_situation(self):
        """Sıkıştığında ne yapılacağını belirle"""
        if self.returning_home:
            # Başlangıç pozisyonuna dönerken sıkıştı
            if self.stuck_count >= self.max_stuck_count:
                self.get_logger().warn("🚫 Başlangıç pozisyonuna dönüş başarısız! Mevcut pozisyonda kalıyor.")
                self.exploration_state = "FINISHED"
                self.returning_home = False
                self.publish_status("RETURN_HOME_FAILED - Staying at current position")
            else:
                self.get_logger().info("🔄 Başlangıç pozisyonuna dönüş yeniden deneniyor...")
                self.return_to_start_position()
        else:
            # Normal exploration sırasında sıkıştı
            if self.stuck_count >= self.max_stuck_count:
                self.get_logger().warn("🚫 Çok fazla stuck! Exploration tamamlandı sayılıyor...")
                if self.return_to_start and self.start_position:
                    self.get_logger().info("🏠 Başlangıç pozisyonuna dönülüyor...")
                    self.return_to_start_position()
                else:
                    self.exploration_state = "FINISHED"
                self.stuck_count = 0
            else:
                self.get_logger().info("🔄 Stuck durumu, yeni hedef aranıyor...")
                self.exploration_state = "EXPLORING"

    def start_exploration(self):
        """Keşfi başlat"""
        self.exploration_state = "EXPLORING"
        self.exploration_start_time = time.time()
        self.get_logger().info("🚀 Exploration başlatıldı!")
        self.publish_status("EXPLORATION_STARTED")

    def distance_to_point(self, point):
        """Robot ile nokta arasındaki mesafe"""
        if not self.robot_position:
            return float('inf')
        
        return math.sqrt(
            (point[0] - self.robot_position.x) ** 2 + 
            (point[1] - self.robot_position.y) ** 2
        )

    def add_explored_goal(self, goal):
        """Keşfedilen hedefi listeye ekle"""
        if goal:
            self.explored_goals.append(goal)
            # Liste çok uzarsa eskilerini sil
            if len(self.explored_goals) > 30:
                self.explored_goals = self.explored_goals[-15:]

    def is_goal_explored(self, goal):
        """Bu hedef daha önce keşfedildi mi?"""
        threshold = 1.5  # 1.5 metre yakınlık
        for explored in self.explored_goals:
            distance = math.sqrt(
                (goal[0] - explored[0]) ** 2 + 
                (goal[1] - explored[1]) ** 2
            )
            if distance < threshold:
                return True
        return False

    def is_ready(self):
        """Exploration için gerekli veriler hazır mı?"""
        return (self.current_map is not None and 
                self.robot_position is not None)

    def enable_exploration_callback(self, request, response):
        """Exploration aktif/pasif service"""
        self.enable_exploration = request.data
        
        if self.enable_exploration:
            self.exploration_state = "WAITING"
            self.returning_home = False
            response.message = "Exploration aktif edildi"
            self.get_logger().info("✅ Exploration aktif edildi")
        else:
            self.exploration_state = "WAITING"
            self.current_goal = None
            self.returning_home = False
            response.message = "Exploration pasif edildi"
            self.get_logger().info("❌ Exploration pasif edildi")
        
        response.success = True
        self.publish_status(f"EXPLORATION_{'ENABLED' if self.enable_exploration else 'DISABLED'}")
        return response

    def publish_status(self, status):
        """Durum mesajı yayınla"""
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def debug_publish(self, message):
        """Debug mesajı yayınla"""
        msg = String()
        msg.data = message
        self.debug_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Exploration Node durduruldu')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()