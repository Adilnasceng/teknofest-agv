#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
import tf_transformations
import math
import time
from threading import Lock
from enum import Enum

# Programın durumlarını tanımlayan Enum sınıfı
class Durum(Enum):
    BOS_BEKLEME = 1
    HEDEF_TANIMLAMA = 2
    NAVIGASYONU_BASLAT = 3
    HEDEFE_GIT = 4
    HEDEF_KONTROL = 5
    EK_HAREKET_BASLAT = 6
    EK_HAREKET_GECIKME = 7    # YENİ DURUM: Gecikme bekleniyor
    EK_HAREKET_KONTROL = 8
    GOREV_SONRASI_BEKLEME = 9  # YENİ DURUM: Görev sonrası bekleme
    BASLANGIC_KONUMA_DON = 10  # YENİ DURUM: Başlangıç konumuna dön
    BASLANGIC_KONUMA_DON_KONTROL = 11  # YENİ DURUM: Başlangıç konumu kontrolü
    GOREV_BITTI = 12
    HATA = 13

class GorevTipi(Enum):
    KUTU_ALMA = "kutu_alma"      # 1, 3, 5 hedefleri için (ileri git)
    KUTU_BIRAKMA = "kutu_birakma" # 2, 4, 6 hedefleri için (geri git)

class CokluGorevYoneticisi(Node):
    def __init__(self):
        super().__init__('dynamic_goal_task_manager')
        self.navigator = BasicNavigator()

        # Parametreler
        self.declare_parameter('total_goals', 6)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('backward_speed', -0.2)
        self.declare_parameter('forward_duration', 3.0)
        self.declare_parameter('backward_duration', 3.0)
        self.declare_parameter('task_delay', 2.0)
        self.declare_parameter('post_task_wait', 5.0)  # Görev sonrası bekleme süresi
        self.declare_parameter('return_to_start', True)  # YENİ: Başlangıça dönüş
        self.declare_parameter('debug_mode', True)
        self.total_goals = self.get_parameter('total_goals').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.backward_duration = self.get_parameter('backward_duration').value
        self.task_delay = self.get_parameter('task_delay').value
        self.post_task_wait = self.get_parameter('post_task_wait').value
        self.return_to_start = self.get_parameter('return_to_start').value  # YENİ

        self.debug_mode = self.get_parameter('debug_mode').value

        # Durum ve görev yönetimi değişkenleri
        self.durum = Durum.HEDEF_TANIMLAMA
        self.current_pose = None
        self.baslangic_pose = None  # YENİ: Başlangıç pozisyonunu kaydet
        self.hedefler = []
        self.hedef_tanimlama_asama = 1
        self.gorev_listesi = []
        self.aktif_gorev_index = 0

        # Görev çalıştırma için değişkenler
        self.is_executing_task = False
        self.current_task = None
        self.task_start_time = 0
        self.task_cmd_vel = Twist()

        # Zaman tabanlı kontroller için
        self.delay_start_time = 0
        self.wait_start_time = 0

        # Thread safety
        self.lock = Lock()
        
        # Service clients - ses için
        self.sound1_client = self.create_client(SetBool, 'play_sound_1')
        self.sound2_client = self.create_client(SetBool, 'play_sound_2')
        self.services_ready = False
        
        # Subscriber'lar
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_status_publisher = self.create_publisher(String, '/task_status', 10)
        self.goal_info_publisher = self.create_publisher(String, '/goal_info', 10)

        # Ana durum makinesi döngüsü için timer
        self.timer = self.create_timer(0.1, self.durum_makinesi_callback)
        self.service_check_timer = self.create_timer(2.0, self.check_services)

        self.get_logger().info("🎯 Çoklu Görev Yöneticisi başlatıldı.")
        self.get_logger().info("📋 Görev sistemi: 1,3,5 → İleri Git | 2,4,6 → Geri Git")
        self.get_logger().info(f"⚡ İleri: {self.forward_speed} m/s ({self.forward_duration}s), Geri: {self.backward_speed} m/s ({self.backward_duration}s)")
        self.get_logger().info(f"⏱️ Görev gecikmesi: {self.task_delay}s, Görev sonrası bekleme: {self.post_task_wait}s")
        self.get_logger().info(f"🎯 Lütfen RViz üzerinden {self.hedef_tanimlama_asama}. hedefi belirleyin.")
        self.publish_task_status(f"GOAL_DEFINITION - Waiting for goal {self.hedef_tanimlama_asama}/{self.total_goals}")

    def check_services(self):
        """Service'lerin hazır olup olmadığını kontrol et"""
        # Removed line_follower_client check since it doesn't exist
        sound1_ready = self.sound1_client.service_is_ready()
        sound2_ready = self.sound2_client.service_is_ready()

        new_status = sound1_ready and sound2_ready
        if new_status != self.services_ready:
            self.services_ready = new_status
            if self.services_ready:
                self.get_logger().info('✅ Ses servisleri hazır!')
            else:
                self.get_logger().warn('⚠️ Ses servisleri bağlantısı yok!')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

        # İlk pose geldiğinde başlangıç pozisyonunu kaydet
        if self.baslangic_pose is None:
            self.baslangic_pose = msg.pose.pose
            x = self.baslangic_pose.position.x
            y = self.baslangic_pose.position.y
            self.get_logger().info(f"🏠 Başlangıç konumu kaydedildi: ({x:.2f}, {y:.2f})")

    def goal_pose_callback(self, msg):
        if self.durum != Durum.HEDEF_TANIMLAMA:
            self.get_logger().warn("❌ Sistem hedef tanımlama modunda değil, yeni hedef alınamıyor.")
            return

        self.hedefler.append(msg)
        x, y = msg.pose.position.x, msg.pose.position.y

        # Görev tipini belirle
        gorev_tipi = self.get_task_type_for_goal(self.hedef_tanimlama_asama)
        gorev_adi = "KUTU ALMA (İleri Git)" if gorev_tipi == GorevTipi.KUTU_ALMA else "KUTU BIRAKMA (Geri Git)"
        self.get_logger().info(f"✅ Hedef {self.hedef_tanimlama_asama} kaydedildi: ({x:.2f}, {y:.2f}) - {gorev_adi}")
        self.publish_goal_info(f"Goal {self.hedef_tanimlama_asama}: ({x:.2f}, {y:.2f}) - {gorev_adi}")

        self.hedef_tanimlama_asama += 1

        if self.hedef_tanimlama_asama > self.total_goals:
            self.get_logger().info(f"🎉 Tüm {self.total_goals} hedef de tanımlandı.")
            self.durum = Durum.NAVIGASYONU_BASLAT
        else:
            self.get_logger().info(f"🎯 Lütfen RViz üzerinden {self.hedef_tanimlama_asama}. hedefi belirleyin.")
            self.publish_task_status(f"GOAL_DEFINITION - Waiting for goal {self.hedef_tanimlama_asama}/{self.total_goals}")

    def get_task_type_for_goal(self, goal_number):
        """Hedef numarasına göre görev tipini belirle"""
        if goal_number % 2 == 1:  # Tek sayılı hedefler (1, 3, 5)
            return GorevTipi.KUTU_ALMA
        else:  # Çift sayılı hedefler (2, 4, 6)
            return GorevTipi.KUTU_BIRAKMA

    def durum_makinesi_callback(self):
        current_time = time.time()

        if self.durum == Durum.HEDEF_TANIMLAMA or self.durum == Durum.BOS_BEKLEME:
            return

        elif self.durum == Durum.NAVIGASYONU_BASLAT:
            self.get_logger().info("🚀 Görev dizisi oluşturuluyor ve başlatılıyor...")
            self.navigator.waitUntilNav2Active()

            # Ana görev listesini oluştur
            for i in range(self.total_goals):
                hedef = self.hedefler[i]
                goal_number = i + 1

                # Görev tipini belirle
                if goal_number % 2 != 0:  # Tek sayılı hedefler
                    gorev_tipi = GorevTipi.KUTU_ALMA
                    ek_hareket = self.forward_speed
                    gorev_adi = "KUTU ALMA (İleri Git)"
                else:  # Çift sayılı hedefler
                    gorev_tipi = GorevTipi.KUTU_BIRAKMA
                    ek_hareket = self.backward_speed
                    gorev_adi = "KUTU BIRAKMA (Geri Git)"
                self.gorev_listesi.append({
                    "hedef_pose": hedef, 
                    "ek_hareket": ek_hareket,
                    "gorev_tipi": gorev_tipi,
                    "isim": f"Hedef {goal_number} ({gorev_adi})"
                })

            self.aktif_gorev_index = 0
            self.durum = Durum.HEDEFE_GIT

        elif self.durum == Durum.HEDEFE_GIT:
            gorev = self.gorev_listesi[self.aktif_gorev_index]
            self.get_logger().info(f"🎯 Görev {self.aktif_gorev_index + 1}/{len(self.gorev_listesi)}: {gorev['isim']}'e gidiliyor...")
            self.publish_task_status(f"NAVIGATING - {gorev['isim']}")
            self.navigator.goToPose(gorev['hedef_pose'])
            self.durum = Durum.HEDEF_KONTROL

        elif self.durum == Durum.HEDEF_KONTROL:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("✅ Hedefe başarıyla ulaşıldı.")
                    self.durum = Durum.EK_HAREKET_BASLAT
                else:
                    self.get_logger().error(f"❌ Hedefe gidilemedi (Durum: {result}).")
                    self.durum = Durum.HATA

        elif self.durum == Durum.EK_HAREKET_BASLAT:
            gorev = self.gorev_listesi[self.aktif_gorev_index]
            if gorev['ek_hareket'] is not None and self.current_pose is not None:
                self.get_logger().info(f"⏱️ {self.task_delay} saniye gecikme başlıyor...")
                self.delay_start_time = current_time
                self.current_task = gorev['gorev_tipi']
                self.publish_task_status(f"TASK_DELAY - Waiting {self.task_delay}s before task")
                self.durum = Durum.EK_HAREKET_GECIKME
            else:
               # Ek hareket yoksa veya pozisyon bilgisi yoksa bir sonraki göreve geç
                self.sonraki_goreve_gec()

        elif self.durum == Durum.EK_HAREKET_GECIKME:
            # Gecikme süresi doldu mu kontrol et
            if current_time - self.delay_start_time >= self.task_delay:
                self.start_task_execution()
                self.durum = Durum.EK_HAREKET_KONTROL
                
        elif self.durum == Durum.EK_HAREKET_KONTROL:
            if self.is_executing_task:
                # Görev sırasında hareket komutları gönder
                self.cmd_vel_publisher.publish(self.task_cmd_vel)
                
                # Görev süresi doldu mu kontrol et
                gorev = self.gorev_listesi[self.aktif_gorev_index]
                task_duration = (self.forward_duration if self.current_task == GorevTipi.KUTU_ALMA 
                               else self.backward_duration)
                elapsed = current_time - self.task_start_time
                if elapsed >= task_duration:
                    self.finish_task_execution()
                    
                # Progress log
                elif self.debug_mode and int(elapsed * 10) % 10 == 0:  # Her saniye log
                    remaining = task_duration - elapsed
                    task_name = "KUTU ALMA" if self.current_task == GorevTipi.KUTU_ALMA else "KUTU BIRAKMA"
                    self.get_logger().info(f'🔄 {task_name} - Kalan: {remaining:.1f}s')
                    
        elif self.durum == Durum.GOREV_SONRASI_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Bekleme süresi doldu mu kontrol et
            if current_time - self.wait_start_time >= self.post_task_wait:
                self.get_logger().info('⏰ Görev sonrası bekleme tamamlandı!')
                self.get_logger().info('🚀 Sıradaki hedefe geçiliyor...')
                self.publish_task_status("POST_TASK_WAIT_COMPLETED - Moving to next goal")
                self.sonraki_goreve_gec()

        elif self.durum == Durum.BASLANGIC_KONUMA_DON:
            if self.baslangic_pose is None:
                self.get_logger().error("❌ Başlangıç konumu bulunamadı!")
                self.durum = Durum.HATA
                return

            # Başlangıç pozisyonunu PoseStamped formatına çevir
            baslangic_goal = PoseStamped()
            baslangic_goal.header.frame_id = 'map'
            baslangic_goal.header.stamp = self.get_clock().now().to_msg()
            baslangic_goal.pose = self.baslangic_pose

            x = self.baslangic_pose.position.x
            y = self.baslangic_pose.position.y
            self.get_logger().info(f"🏠 Başlangıç konumuna gidiliyor: ({x:.2f}, {y:.2f})")
            self.publish_task_status(f"RETURNING_HOME - Going to start position ({x:.2f}, {y:.2f})")

            # Başlangıç konumuna navigasyon başlat
            self.navigator.goToPose(baslangic_goal)
            self.durum = Durum.BASLANGIC_KONUMA_DON_KONTROL

        elif self.durum == Durum.BASLANGIC_KONUMA_DON_KONTROL:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("🏠✅ Başlangıç konumuna başarıyla döndü!")
                    self.publish_task_status("RETURNED_HOME - Successfully returned to start position")
                    self.durum = Durum.GOREV_BITTI
                else:
                    self.get_logger().error(f"❌ Başlangıç konumuna dönülemedi (Durum: {result}).")
                    self.publish_task_status("RETURN_HOME_FAILED - Could not return to start position")
                    self.durum = Durum.HATA

        elif self.durum == Durum.GOREV_BITTI or self.durum == Durum.HATA:
            if self.durum == Durum.GOREV_BITTI: 
                self.get_logger().info("🎉 Tüm görevler başarıyla tamamlandı ve başlangıç konumuna döndü!")
            else: 
                self.get_logger().info("❌ Görev dizisi bir hatadan dolayı sonlandı.")

            # Sistemi sıfırla
            self.hedefler = []
            self.gorev_listesi = []
            self.hedef_tanimlama_asama = 1
            self.aktif_gorev_index = 0
            self.baslangic_pose = None  # Başlangıç pozisyonunu sıfırla
            self.durum = Durum.HEDEF_TANIMLAMA
            self.get_logger().info("🔄 Sistem yeni görevler için hazır.")
            self.get_logger().info(f"🎯 Lütfen RViz üzerinden {self.hedef_tanimlama_asama}. hedefi belirleyin.")
            self.publish_task_status(f"SYSTEM_RESET - Waiting for goal {self.hedef_tanimlama_asama}/{self.total_goals}")

    def sonraki_goreve_gec(self):
        self.aktif_gorev_index += 1
        if self.aktif_gorev_index < len(self.gorev_listesi):
            self.durum = Durum.HEDEFE_GIT
        else:
            # Tüm görevler tamamlandı
            if self.return_to_start:
                # Başlangıç konumuna dön
                self.get_logger().info("✅ Tüm görevler tamamlandı! Başlangıç konumuna dönülüyor...")
                self.durum = Durum.BASLANGIC_KONUMA_DON
            else:
                # Direkt bitir
                self.get_logger().info("✅ Tüm görevler tamamlandı!")
                self.durum = Durum.GOREV_BITTI

    def start_task_execution(self):
        """Görev çalıştırma başlat"""
        with self.lock:
            if self.is_executing_task:
                self.get_logger().warn('⚠️ Zaten bir görev çalışıyor!')
                return

            gorev = self.gorev_listesi[self.aktif_gorev_index]
            
            # Görev parametrelerini ayarla
            if gorev['gorev_tipi'] == GorevTipi.KUTU_ALMA:
                task_duration = self.forward_duration
                self.task_cmd_vel.linear.x = self.forward_speed
                task_name = "KUTU ALMA (İLERİ GİT)"
            else:
                task_duration = self.backward_duration
                self.task_cmd_vel.linear.x = self.backward_speed
                task_name = "KUTU BIRAKMA (GERİ GİT)"
            
            # Diğer hareket bileşenlerini sıfırla
            self.task_cmd_vel.linear.y = 0.0
            self.task_cmd_vel.linear.z = 0.0
            self.task_cmd_vel.angular.x = 0.0
            self.task_cmd_vel.angular.y = 0.0
            self.task_cmd_vel.angular.z = 0.0
            self.is_executing_task = True
            self.task_start_time = time.time()

            self.get_logger().info(f'🚀 GÖREV BAŞLATILDI: {task_name}')
            self.get_logger().info(f'⚡ Hız: {self.task_cmd_vel.linear.x} m/s, Süre: {task_duration}s')
            self.publish_task_status(f"TASK_EXECUTING - {task_name} - Duration: {task_duration}s")
            # Göreve özel ses çal
            self.play_task_sound()
            
    def finish_task_execution(self):
        """Görev çalıştırma bitir"""
        with self.lock:
            if not self.is_executing_task:
                return

            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

            task_name = "KUTU ALMA" if self.current_task == GorevTipi.KUTU_ALMA else "KUTU BIRAKMA"
            self.get_logger().info(f'✅ GÖREV TAMAMLANDI: {task_name}')
            self.get_logger().info(f'⏳ {self.post_task_wait} saniye bekleme başlıyor...')

            self.publish_task_status(f"TASK_COMPLETED - {task_name} - Waiting {self.post_task_wait}s")

            # Görev durumunu sıfırla
            self.is_executing_task = False
            self.current_task = None
            self.task_cmd_vel = Twist()

            # Post-task bekleme başlat
            self.wait_start_time = time.time()
            self.durum = Durum.GOREV_SONRASI_BEKLEME
            
    def play_task_sound(self):
        """Görev tipine göre ses çal"""
        if not self.services_ready:
            return

        try:
            request = SetBool.Request()
            request.data = True

            if self.current_task == GorevTipi.KUTU_ALMA:
                # Kutu alma için ses 1
                future = self.sound1_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, "kutu_alma"))
            else:
                # Kutu bırakma için ses 2
                future = self.sound2_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, "kutu_birakma"))

        except Exception as e:
            self.get_logger().error(f'Görev sesi çalma hatası: {e}')

    def sound_callback(self, future, task_type):
        """Ses çalma callback"""
        try:
            response = future.result()
            if response.success:
                sound_name = "kutu alma sesi" if task_type == "kutu_alma" else "kutu bırakma sesi"
                self.get_logger().info(f'🔊 {sound_name} çalındı! 🎵')
            else:
                self.get_logger().error(f'❌ {task_type} sesi çalma başarısız')
        except Exception as e:
            self.get_logger().error(f'{task_type} ses callback hatası: {e}')

    def publish_task_status(self, status):
        """Görev durumunu yayınla"""
        status_msg = String()
        status_msg.data = status
        self.task_status_publisher.publish(status_msg)

    def publish_goal_info(self, info):
        """Hedef bilgisini yayınla"""
        info_msg = String()
        info_msg.data = info
        self.goal_info_publisher.publish(info_msg)

    def emergency_stop(self):
        """Acil durdurma"""
        with self.lock:
            if self.is_executing_task:
                stop_cmd = Twist()
                self.cmd_vel_publisher.publish(stop_cmd)
                self.is_executing_task = False
                self.current_task = None
                self.task_cmd_vel = Twist()

                self.get_logger().warn('🛑 ACİL DURDURMA - Tüm görevler iptal edildi!')
                self.publish_task_status("EMERGENCY_STOP - All tasks cancelled")

def main(args=None):
    rclpy.init(args=args)
    node = CokluGorevYoneticisi()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('Çoklu Görev Yöneticisi durduruldu')
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()