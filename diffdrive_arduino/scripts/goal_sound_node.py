#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.msg import Log
import time
from threading import Lock

class GoalSoundNode(Node):
    def __init__(self):
        super().__init__('goal_sound_node')
        
        # Parametreler
        self.declare_parameter('enable_goal_sounds', True)
        self.declare_parameter('sound_delay', 1.0)
        self.declare_parameter('debug_mode', True)
        
        self.enable_goal_sounds = self.get_parameter('enable_goal_sounds').value
        self.sound_delay = self.get_parameter('sound_delay').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Thread safety
        self.lock = Lock()
        
        # Goal tracking
        self.completed_goals = 0
        self.last_goal_time = 0
        self.processed_goals = set()
        self.pending_sounds = []
        
        # Service clients
        self.sound1_client = self.create_client(SetBool, 'play_sound_1')
        self.sound2_client = self.create_client(SetBool, 'play_sound_2')
        self.services_ready = False
        
        # QoS profili
        action_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Nav2 action status'larını izle
        self.navigate_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.navigate_status_callback,
            action_qos
        )
        
        # BT Navigator mesajlarını izle
        self.log_sub = self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            100
        )
        
        # Goal pose backup
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        # Timer'lar
        self.service_check_timer = self.create_timer(2.0, self.check_services)
        self.sound_queue_timer = self.create_timer(0.1, self.process_sound_queue)
        
        # Waypoint tracking
        self.last_bt_success_time = 0
        self.bt_success_debounce = 3.0  # BT success mesajları arası minimum süre
        
        self.get_logger().info('🎵 Goal Sound Node başlatıldı (Final Fix)')
        self.get_logger().info('📍 BT Navigator "Goal succeeded" mesajlarını izliyor')
        self.get_logger().info(f'🔊 Ses sistemi: {"AKTIF" if self.enable_goal_sounds else "PASIF"}')

    def check_services(self):
        """Service'lerin hazır olup olmadığını kontrol et"""
        sound1_ready = self.sound1_client.service_is_ready()
        sound2_ready = self.sound2_client.service_is_ready()
        
        new_status = sound1_ready and sound2_ready
        
        if new_status != self.services_ready:
            self.services_ready = new_status
            if self.services_ready:
                self.get_logger().info('✅ Ses servisleri hazır!')
            else:
                self.get_logger().warn('⚠️ Ses servisleri bağlantısı yok!')

    def log_callback(self, msg):
        """ROS log mesajlarını izle"""
        if not self.enable_goal_sounds:
            return
        
        current_time = time.time()
        
        # BT Navigator'dan "Goal succeeded" mesajlarını yakala
        if (msg.name == 'bt_navigator' and 
            'Goal succeeded' in msg.msg):
            
            # Debounce check - waypoint'lerde her hedef için ayrı "Goal succeeded" gelir
            if current_time - self.last_bt_success_time >= self.bt_success_debounce:
                self.last_bt_success_time = current_time
                
                if self.debug_mode:
                    self.get_logger().info('🎯 BT Navigator: Goal succeeded (Log-based)')
                
                self.goal_completed("BT-Navigator")

    def goal_pose_callback(self, msg):
        """Goal pose alındığında"""
        current_time = time.time()
        self.last_goal_time = current_time
        
        if self.debug_mode:
            x = msg.pose.position.x
            y = msg.pose.position.y
            self.get_logger().info(f'🎯 Yeni hedef alındı: ({x:.2f}, {y:.2f})')

    def navigate_status_callback(self, msg):
        """NavigateToPose action status callback - sadece debug için"""
        if not self.enable_goal_sounds:
            return
            
        for status in msg.status_list:
            goal_id = status.goal_info.goal_id.uuid
            goal_id_str = ''.join([f'{b:02x}' for b in goal_id])
            
            if self.debug_mode:
                status_text = self.get_status_text(status.status)
                self.get_logger().info(f'📍 Navigate status: {status_text} (ID: {goal_id_str[:8]}...)')
            
            # Action-based ses çalmayı devre dışı bırak - sadece BT Navigator log'u kullan
            # if status.status == 4 and goal_id_str not in self.processed_goals:
            #     self.processed_goals.add(goal_id_str)
            #     self.goal_completed("NavigateToPose")

    def get_status_text(self, status):
        """Action status'unu metin haline çevir"""
        status_map = {
            0: "UNKNOWN",
            1: "ACCEPTED", 
            2: "EXECUTING",
            3: "CANCELING",
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED"
        }
        return status_map.get(status, f"STATUS_{status}")

    def goal_completed(self, source):
        """Hedef tamamlandığında çağrılır"""
        with self.lock:
            self.completed_goals += 1
            
            # Hangi sesi çalacağını belirle (2→1→2→1...)
            sound_number = 2 if ((self.completed_goals - 1) % 2) == 0 else 1
            
            self.get_logger().info(f'🎉 Hedef #{self.completed_goals} tamamlandı! (Kaynak: {source})')
            self.get_logger().info(f'🎵 Ses {sound_number} çalınacak...')
            
            # Ses çalmayı kuyruğa ekle
            play_time = time.time() + self.sound_delay
            self.pending_sounds.append((play_time, sound_number))

    def process_sound_queue(self):
        """Bekleyen sesleri işle"""
        if not self.pending_sounds:
            return
            
        current_time = time.time()
        sounds_to_play = []
        
        # Zamanı gelen sesleri bul
        for i, (play_time, sound_number) in enumerate(self.pending_sounds):
            if current_time >= play_time:
                sounds_to_play.append((i, sound_number))
        
        # Sesleri çal ve kuyruktan çıkar
        for i in reversed(range(len(sounds_to_play))):
            index, sound_number = sounds_to_play[i]
            self.play_sound(sound_number)
            self.pending_sounds.pop(index)

    def play_sound(self, sound_number):
        """Belirtilen sesi çal"""
        if not self.services_ready:
            self.get_logger().warn('🚫 Ses servisleri hazır değil!')
            return
        
        try:
            request = SetBool.Request()
            request.data = True
            
            if sound_number == 1:
                future = self.sound1_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, 1))
                self.get_logger().info('🔊 Ses 1 çalınıyor...')
            else:
                future = self.sound2_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, 2))
                self.get_logger().info('🔊 Ses 2 çalınıyor...')
                
        except Exception as e:
            self.get_logger().error(f'Ses çalma hatası: {e}')

    def sound_callback(self, future, sound_number):
        """Ses çalma service callback"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Ses {sound_number} başarıyla çalındı! 🎵')
            else:
                self.get_logger().error(f'❌ Ses {sound_number} çalma başarısız: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Ses {sound_number} callback hatası: {e}')

    def reset_goal_count(self):
        """Hedef sayacını sıfırla"""
        with self.lock:
            self.completed_goals = 0
            self.processed_goals.clear()
            self.pending_sounds.clear()
            self.last_bt_success_time = 0
            self.get_logger().info('🔄 Hedef sayacı ve cache sıfırlandı')

    def manual_goal_completed(self):
        """Manuel olarak goal completed sinyali gönder"""
        self.get_logger().info('📞 Manuel goal completed sinyali')
        self.goal_completed("Manual")


def main(args=None):
    rclpy.init(args=args)
    node = GoalSoundNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Goal Sound Node durduruldu')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()