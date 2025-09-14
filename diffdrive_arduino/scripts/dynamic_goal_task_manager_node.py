#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool, Trigger
import tf_transformations
import math
import time
from threading import Lock
from enum import Enum

# Programın durumlarını tanımlayan Enum sınıfı
class Durum(Enum):
    BASLANGIC_KONUMU_TANIMLAMA = 0  # Manuel başlangıç konumu tanımlama
    SARJ_ISTASYONU_TANIMLAMA = 1    # Şarj istasyonu tanımlama
    BOS_BEKLEME = 2
    HEDEF_TANIMLAMA = 3
    NAVIGASYONU_BASLAT = 4
    HEDEFE_GIT = 5
    HEDEF_KONTROL = 6
    EK_HAREKET_BASLAT = 7
    EK_HAREKET_GECIKME = 8    # Gecikme bekleniyor
    EK_HAREKET_KONTROL = 9
    YONLENME_BEKLEME = 10     # Yönlenme görevi bekleme
    # Servo ve ses kontrolü için durumlar
    SERVO_ONCESI_BEKLEME = 11     # Servo tetiklemeden önce bekleme
    SERVO_TETIKLEME = 12          # Servo tetikleme
    SERVO_SONRASI_BEKLEME = 13    # Servo tetikleme sonrası bekleme
    SES_ONCESI_BEKLEME = 14       # Ses çalmadan önce bekleme
    SES_SONRASI_BEKLEME = 15      # Ses sonrası bekleme
    GOREV_SONRASI_BEKLEME = 16    # Final görev sonrası bekleme
    BASLANGIC_KONUMA_DON = 17     # Başlangıç konumuna dön
    BASLANGIC_KONUMA_DON_KONTROL = 18  # Başlangıç konumu kontrolü
    # Şarj istasyonu durumları
    SARJ_ISTASYONUNA_GIT = 19         # Şarj istasyonuna git
    SARJ_ISTASYONU_KONTROL = 20       # Şarj istasyonu navigasyon kontrolü
    SARJ_BEKLEME = 21                 # Şarj istasyonunda bekleme
    # YENİ: Görevler tamamlandıktan sonra bekleme ve şarj kontrolü
    BEKLEME_VE_SARJ_KONTROL = 22      # Başlangıçta bekle ve batarya kontrol et
    HATA = 23

class GorevTipi(Enum):
    YONLENME = "yonlenme"        # Sadece hedefe git ve bekle
    KUTU_ALMA = "kutu_alma"      # Kutu alma görevleri için (ileri git)
    KUTU_BIRAKMA = "kutu_birakma" # Kutu bırakma görevleri için (geri git)

class CokluGorevYoneticisi(Node):
    def __init__(self):
        super().__init__('enhanced_task_manager')
        self.navigator = BasicNavigator()

        # Parametreler
        self.declare_parameter('base_goals', 2)  # 2 temel görev = 4 toplam hedef
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('backward_speed', -0.2)
        self.declare_parameter('forward_duration', 3.0)
        self.declare_parameter('backward_duration', 3.0)
        self.declare_parameter('task_delay', 2.0)
        self.declare_parameter('navigation_wait', 5.0)  # Yönlenme bekleme süresi
        
        # Servo ve ses kontrol parametreleri
        self.declare_parameter('pre_servo_wait', 5.0)     # Servo tetiklemeden önce bekleme
        self.declare_parameter('post_servo_wait', 5.0)    # Servo tetikleme sonrası bekleme  
        self.declare_parameter('pre_sound_wait', 5.0)     # Ses çalmadan önce bekleme
        self.declare_parameter('post_sound_wait', 5.0)    # Ses sonrası bekleme
        self.declare_parameter('post_task_wait', 5.0)     # Final görev sonrası bekleme süresi
        self.declare_parameter('enable_servo_control', True)  # Servo kontrol aktif/pasif
        self.declare_parameter('enable_sound_control', True)  # Ses kontrol aktif/pasif
        
        self.declare_parameter('return_to_start', True) # Başlangıça dönüş
        self.declare_parameter('debug_mode', True)
        # Engel algılama kontrolü parametreleri
        self.declare_parameter('enable_obstacle_control', True)  # Engel algılama kontrolü aktif/pasif
        
        # Şarj istasyonu parametreleri
        self.declare_parameter('min_battery_level', 25.0)      # Minimum batarya seviyesi (%)
        self.declare_parameter('full_battery_level', 95.0)     # Şarj tamamlanmış seviyesi (%)
        self.declare_parameter('battery_check_interval', 5.0)  # Batarya kontrolü aralığı (saniye)
        self.declare_parameter('charging_wait_time', 10.0)     # Şarj istasyonunda bekleme süresi (saniye)
        self.declare_parameter('enable_battery_management', True)  # Batarya yönetimi aktif/pasif
        
        self.base_goals = self.get_parameter('base_goals').value
        self.total_goals = self.base_goals * 2  # Her temel görev için 2 hedef
        self.forward_speed = self.get_parameter('forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.backward_duration = self.get_parameter('backward_duration').value
        self.task_delay = self.get_parameter('task_delay').value
        self.navigation_wait = self.get_parameter('navigation_wait').value
        
        # Servo ve ses kontrol parametreleri
        self.pre_servo_wait = self.get_parameter('pre_servo_wait').value
        self.post_servo_wait = self.get_parameter('post_servo_wait').value
        self.pre_sound_wait = self.get_parameter('pre_sound_wait').value
        self.post_sound_wait = self.get_parameter('post_sound_wait').value
        self.post_task_wait = self.get_parameter('post_task_wait').value
        self.enable_servo_control = self.get_parameter('enable_servo_control').value
        self.enable_sound_control = self.get_parameter('enable_sound_control').value
        
        self.return_to_start = self.get_parameter('return_to_start').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.enable_obstacle_control = self.get_parameter('enable_obstacle_control').value
        
        # Şarj istasyonu parametreleri
        self.min_battery_level = self.get_parameter('min_battery_level').value
        self.full_battery_level = self.get_parameter('full_battery_level').value
        self.battery_check_interval = self.get_parameter('battery_check_interval').value
        self.charging_wait_time = self.get_parameter('charging_wait_time').value
        self.enable_battery_management = self.get_parameter('enable_battery_management').value

        # Durum ve görev yönetimi değişkenleri
        self.durum = Durum.BASLANGIC_KONUMU_TANIMLAMA  # Başlangıç konumu tanımlama ile başla
        self.current_pose = None
        self.baslangic_pose = None  # Manuel olarak tanımlanacak başlangıç pozisyonu
        self.baslangic_konumu_tanimlandi = False  # Başlangıç konumunun tanımlandığını takip et
        # Şarj istasyonu değişkenleri
        self.sarj_istasyonu_pose = None  # Şarj istasyonu pozisyonu
        self.sarj_istasyonu_tanimlandi = False  # Şarj istasyonu tanımlandı mı?
        self.current_battery_percentage = 100.0  # Mevcut batarya yüzdesi
        self.last_battery_check_time = 0  # Son batarya kontrol zamanı
        self.sarj_oncesi_durum = None  # Şarja gitmeden önceki durum
        self.sarj_oncesi_gorev_index = 0  # Şarja gitmeden önceki görev indeksi
        self.sarj_bekleme_start_time = 0  # Şarj bekleme başlangıç zamanı
        
        self.hedefler = []
        self.hedef_tanimlama_asama = 1
        self.gorev_listesi = []
        self.aktif_gorev_index = 0
        
        # YENİ: Görevlerin tamamlanıp tamamlanmadığını takip et
        self.gorevler_tamamlandi = False  # Tüm görevler bir kere tamamlandı mı?

        # Görev çalıştırma için değişkenler
        self.is_executing_task = False
        self.current_task = None
        self.completed_task_type = None  # Tamamlanan görev tipini sakla
        self.task_start_time = 0
        self.task_cmd_vel = Twist()

        # Zaman tabanlı kontroller için
        self.delay_start_time = 0
        self.wait_start_time = 0
        self.navigation_wait_start_time = 0  # Yönlenme bekleme zamanı
        
        # Servo ve ses kontrol zamanları
        self.servo_wait_start_time = 0
        self.sound_wait_start_time = 0

        # Engel algılama kontrolü durumu
        self.obstacle_detection_active = False

        # Thread safety
        self.lock = Lock()
        
        # Service clients - ses ve servo için
        self.sound1_client = self.create_client(SetBool, 'play_sound_1')
        self.sound2_client = self.create_client(SetBool, 'play_sound_2')
        # Servo kontrol client
        self.servo_client = self.create_client(Trigger, '/trigger_servo')
        
        self.services_ready = False
        self.servo_service_ready = False  # Servo service durumu
        
        # Subscriber'lar
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        # Batarya durumu subscriber'ı
        self.battery_sub = self.create_subscription(BatteryState, '/battery_status', self.battery_callback, 10)
        # Minimum batarya seviyesi setter subscriber'ı
        self.min_battery_sub = self.create_subscription(Float32, '/set_min_battery_level', self.set_min_battery_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.task_status_publisher = self.create_publisher(String, '/task_status', 10)
        self.goal_info_publisher = self.create_publisher(String, '/goal_info', 10)
        # Batarya durumu publisher'ı
        self.battery_info_publisher = self.create_publisher(String, '/battery_info', 10)
        
        # Engel algılama kontrolü için publisher
        self.obstacle_control_publisher = self.create_publisher(Bool, '/obstacle_wait_enable', 10)

        # Ana durum makinesi döngüsü için timer
        self.timer = self.create_timer(0.1, self.durum_makinesi_callback)
        self.service_check_timer = self.create_timer(2.0, self.check_services)
        # Batarya kontrol timer'ı
        self.battery_check_timer = self.create_timer(self.battery_check_interval, self.check_battery_level)

        self.get_logger().info("🎯 Gelişmiş Çoklu Görev Yöneticisi başlatıldı - TEK SEFERLİK GÖREV MODU")
        self.get_logger().info("📋 Görev sistemi: Yönlenme → Kutu Alma → Yönlenme → Kutu Bırakma")
        self.get_logger().info(f"🔢 {self.base_goals} temel görev = {self.total_goals} toplam hedef")
        self.get_logger().info("🔄 Görevler 1 kere tamamlandıktan sonra sadece şarj kontrolü yapılacak")
        self.get_logger().info(f"⚡ İleri: {self.forward_speed} m/s ({self.forward_duration}s), Geri: {self.backward_speed} m/s ({self.backward_duration}s)")
        self.get_logger().info(f"⏱️ Yönlenme bekleme: {self.navigation_wait}s, Görev gecikmesi: {self.task_delay}s")
        
        # Servo ve ses kontrol log'ları
        if self.enable_servo_control:
            self.get_logger().info(f"🤖 Servo kontrolü AKTİF - Öncesi: {self.pre_servo_wait}s, Sonrası: {self.post_servo_wait}s")
        else:
            self.get_logger().info("🤖 Servo kontrolü PASİF")
            
        if self.enable_sound_control:
            self.get_logger().info(f"🔊 Ses kontrolü AKTİF - Öncesi: {self.pre_sound_wait}s, Sonrası: {self.post_sound_wait}s")
        else:
            self.get_logger().info("🔊 Ses kontrolü PASİF")
            
        self.get_logger().info(f"⏰ Final görev sonrası bekleme: {self.post_task_wait}s")
        
        # Engel algılama kontrolü log'u
        if self.enable_obstacle_control:
            self.get_logger().info("🚧 Engel algılama kontrolü AKTİF - Yönlenme görevlerinde çalışacak")
        else:
            self.get_logger().info("🚧 Engel algılama kontrolü PASİF")
            
        # Batarya yönetimi log'ları
        if self.enable_battery_management:
            self.get_logger().info(f"🔋 Batarya yönetimi AKTİF - Min: {self.min_battery_level}%, Tam: {self.full_battery_level}%")
            self.get_logger().info(f"🔋 Kontrol aralığı: {self.battery_check_interval}s, Şarj bekleme: {self.charging_wait_time}s")
            self.get_logger().info("📡 Minimum batarya seviyesini değiştirmek için:")
            self.get_logger().info("   ros2 topic pub --once /set_min_battery_level std_msgs/Float32 \"data: 30.0\"")
        else:
            self.get_logger().info("🔋 Batarya yönetimi PASİF")
            
        # İlk olarak başlangıç konumu tanımlama talimatı
        self.get_logger().info("🏠 ÖNCE RViz üzerinden BAŞLANGIÇ KONUMUNU belirleyin.")
        self.publish_task_status("START_POSITION_DEFINITION - Set start position first")
        # Retry mekanizması
        self.max_retries = 3
        self.current_retry_count = 0
        # Başlangıçta engel algılamayı kapat
        self.set_obstacle_detection(False)
        # Başlangıçta 1 kere servo tetikleme
        self.startup_servo_done = False
        self.startup_timer = self.create_timer(2.0, self.startup_servo_check)

    def startup_servo_check(self):
        if not self.startup_servo_done and self.servo_service_ready:
            self.startup_servo_done = True
            self.startup_timer.destroy()
        
            request = Trigger.Request()
            self.servo_client.call_async(request)
            
    # Batarya callback'i
    def battery_callback(self, msg):
        """Batarya durumunu güncelle"""
        self.current_battery_percentage = msg.percentage * 100.0  # 0-1 aralığından 0-100'e çevir
        
        if self.debug_mode and time.time() - self.last_battery_check_time > 30:  # Her 30 saniyede bir debug log
            self.get_logger().info(f"🔋 Batarya: {self.current_battery_percentage:.1f}%")

    # Minimum batarya seviyesi setter callback'i
    def set_min_battery_callback(self, msg):
        """Minimum batarya seviyesini güncelle"""
        old_level = self.min_battery_level
        self.min_battery_level = msg.data
        
        self.get_logger().info(f"🔋 Minimum batarya seviyesi güncellendi: {old_level:.1f}% → {self.min_battery_level:.1f}%")
        self.publish_battery_info(f"Min battery level updated: {self.min_battery_level:.1f}%")

    # Batarya seviyesi kontrolü
    def check_battery_level(self):
        """Batarya seviyesini kontrol et ve gerektiğinde şarj istasyonuna yönlendir"""
        if not self.enable_battery_management:
            return
            
        current_time = time.time()
        self.last_battery_check_time = current_time
        
        # Batarya seviyesi düşükse ve şarj istasyonu tanımlandıysa ve kutu bırakma görevinde değilse
        if (self.current_battery_percentage < self.min_battery_level and 
            self.sarj_istasyonu_tanimlandi and
            not self.is_in_critical_task()):
            
            self.get_logger().warn(f"⚠️ Batarya seviyesi düşük: {self.current_battery_percentage:.1f}% < {self.min_battery_level:.1f}%")
            self.get_logger().info("🔌 Şarj istasyonuna yönlendiriliyor...")
            
            # Mevcut durumu kaydet
            self.sarj_oncesi_durum = self.durum
            self.sarj_oncesi_gorev_index = self.aktif_gorev_index
            
            # Şarj istasyonuna git
            self.durum = Durum.SARJ_ISTASYONUNA_GIT
            self.publish_task_status(f"LOW_BATTERY - Going to charging station ({self.current_battery_percentage:.1f}%)")
            
    def is_in_critical_task(self):
        """Kritik görev durumunda mı kontrol et (kutu bırakma sırasında şarja gitmesin)"""
        # Kutu bırakma görevi sırasında şarja gitmesin
        if (self.is_executing_task and 
            self.current_task == GorevTipi.KUTU_BIRAKMA):
            return True
            
        # Servo veya ses kontrolleri sırasında şarja gitmesin
        if self.durum in [Durum.SERVO_ONCESI_BEKLEME, Durum.SERVO_TETIKLEME, 
                          Durum.SERVO_SONRASI_BEKLEME, Durum.SES_ONCESI_BEKLEME, 
                          Durum.SES_SONRASI_BEKLEME]:
            return True
            
        return False

    def check_services(self):
        """Service'lerin hazır olup olmadığını kontrol et"""
        sound1_ready = self.sound1_client.service_is_ready()
        sound2_ready = self.sound2_client.service_is_ready()
        servo_ready = self.servo_client.service_is_ready()

        new_sound_status = sound1_ready and sound2_ready
        if new_sound_status != self.services_ready:
            self.services_ready = new_sound_status
            if self.services_ready:
                self.get_logger().info('✅ Ses servisleri hazır!')
            else:
                self.get_logger().warn('⚠️ Ses servisleri bağlantısı yok!')
                
        # Servo service durumu
        if servo_ready != self.servo_service_ready:
            self.servo_service_ready = servo_ready
            if self.servo_service_ready:
                self.get_logger().info('✅ Servo servisi hazır!')
            else:
                self.get_logger().warn('⚠️ Servo servisi bağlantısı yok!')

    def pose_callback(self, msg):
        # Sadece mevcut pozisyonu güncelle, otomatik başlangıç kaydı yapma
        self.current_pose = msg.pose.pose

    def goal_pose_callback(self, msg):
        # YENİ: Görevler tamamlandıktan sonra yeni görev tanımlama kabul etme
        if self.gorevler_tamamlandi:
            self.get_logger().warn("❌ Tüm görevler tamamlandı, yeni görev kabul edilmiyor.")
            self.get_logger().info("🔋 Sadece batarya kontrolü yapılıyor.")
            return
            
        # Önce başlangıç konumu tanımlama kontrolü
        if self.durum == Durum.BASLANGIC_KONUMU_TANIMLAMA:
            if not self.baslangic_konumu_tanimlandi:
                # Başlangıç konumunu manuel olarak tanımla
                self.baslangic_pose = msg.pose
                self.baslangic_konumu_tanimlandi = True
                
                x, y = msg.pose.position.x, msg.pose.position.y
                self.get_logger().info(f"🏠✅ Başlangıç konumu manuel olarak tanımlandı: ({x:.2f}, {y:.2f})")
                self.publish_goal_info(f"Start Position: ({x:.2f}, {y:.2f})")
                
                # Şarj istasyonu tanımlama durumuna geç
                self.durum = Durum.SARJ_ISTASYONU_TANIMLAMA
                
                # Şarj istasyonu tanımlama talimatı
                if self.enable_battery_management:
                    self.get_logger().info("🔌 Şimdi RViz üzerinden ŞARJ İSTASYONU konumunu belirleyin.")
                    self.publish_task_status("CHARGING_STATION_DEFINITION - Set charging station position")
                else:
                    self.get_logger().info("🔋 Batarya yönetimi devre dışı, hedef tanımlama aşamasına geçiliyor.")
                    self.durum = Durum.HEDEF_TANIMLAMA
                    self.start_goal_definition()
                return
            else:
                self.get_logger().warn("❌ Başlangıç konumu zaten tanımlandı.")
                return

        # Şarj istasyonu tanımlama kontrolü
        elif self.durum == Durum.SARJ_ISTASYONU_TANIMLAMA:
            if not self.sarj_istasyonu_tanimlandi:
                # Şarj istasyonu konumunu manuel olarak tanımla
                self.sarj_istasyonu_pose = msg.pose
                self.sarj_istasyonu_tanimlandi = True
                
                x, y = msg.pose.position.x, msg.pose.position.y
                self.get_logger().info(f"🔌✅ Şarj istasyonu konumu manuel olarak tanımlandı: ({x:.2f}, {y:.2f})")
                self.publish_goal_info(f"Charging Station: ({x:.2f}, {y:.2f})")
                
                # Hedef tanımlama durumuna geç
                self.durum = Durum.HEDEF_TANIMLAMA
                self.start_goal_definition()
                return
            else:
                self.get_logger().warn("❌ Şarj istasyonu konumu zaten tanımlandı.")
                return

        elif self.durum != Durum.HEDEF_TANIMLAMA:
            self.get_logger().warn("❌ Sistem hedef tanımlama modunda değil, yeni hedef alınamıyor.")
            return

        # Hedef tanımlama işlemi (eski kod)
        self.hedefler.append(msg)
        x, y = msg.pose.position.x, msg.pose.position.y

        # Görev tipini belirle
        gorev_tipi = self.get_task_type_for_goal(self.hedef_tanimlama_asama)
        gorev_adi = self.get_task_description(gorev_tipi)
        
        self.get_logger().info(f"✅ Hedef {self.hedef_tanimlama_asama} kaydedildi: ({x:.2f}, {y:.2f}) - {gorev_adi}")
        self.publish_goal_info(f"Goal {self.hedef_tanimlama_asama}: ({x:.2f}, {y:.2f}) - {gorev_adi}")

        self.hedef_tanimlama_asama += 1

        if self.hedef_tanimlama_asama > self.total_goals:
            self.get_logger().info(f"🎉 Tüm {self.total_goals} hedef de tanımlandı.")
            self.durum = Durum.NAVIGASYONU_BASLAT
        else:
            next_task_type = self.get_task_type_for_goal(self.hedef_tanimlama_asama)
            next_task_desc = self.get_task_description(next_task_type)
            self.get_logger().info(f"🎯 Lütfen RViz üzerinden {self.hedef_tanimlama_asama}. hedefi belirleyin ({next_task_desc}).")
            self.publish_task_status(f"GOAL_DEFINITION - Waiting for goal {self.hedef_tanimlama_asama}/{self.total_goals} ({next_task_desc})")

    def start_goal_definition(self):
        """Hedef tanımlama sürecini başlat"""
        next_task_type = self.get_task_type_for_goal(self.hedef_tanimlama_asama)
        next_task_desc = self.get_task_description(next_task_type)
        self.get_logger().info(f"🎯 Şimdi RViz üzerinden {self.hedef_tanimlama_asama}. hedefi belirleyin ({next_task_desc}).")
        self.publish_task_status(f"GOAL_DEFINITION - Waiting for goal {self.hedef_tanimlama_asama}/{self.total_goals} ({next_task_desc})")

    def get_task_type_for_goal(self, goal_number):
        """Hedef numarasına göre görev tipini belirle - YENİ SISTEM"""
        # 1-2: Yönlenme-Kutu Alma, 3-4: Yönlenme-Kutu Bırakma, 5-6: Yönlenme-Kutu Alma, vs.
        if goal_number % 2 == 1:  # Tek sayılı hedefler → Yönlenme
            return GorevTipi.YONLENME
        else:  # Çift sayılı hedefler → Kutu işlemi
            # Hangi çift grup olduğuna göre kutu alma/bırakma belirle
            group_number = (goal_number // 2)  # 1. grup: 2, 2. grup: 4, 3. grup: 6, vs.
            if group_number % 2 == 1:  # 1., 3., 5. grup → Kutu Alma
                return GorevTipi.KUTU_ALMA
            else:  # 2., 4., 6. grup → Kutu Bırakma
                return GorevTipi.KUTU_BIRAKMA

    def get_task_description(self, gorev_tipi):
        """Görev tipine göre açıklama döndür"""
        if gorev_tipi == GorevTipi.YONLENME:
            return "YÖNLENME (Git ve Bekle)"
        elif gorev_tipi == GorevTipi.KUTU_ALMA:
            return "KUTU ALMA (İleri Git)"
        else:
            return "KUTU BIRAKMA (Geri Git)"

    def set_obstacle_detection(self, enable):
        """Engel algılama durumunu ayarla"""
        if not self.enable_obstacle_control:
            return  # Kontrol devre dışıysa hiçbir şey yapma
            
        if self.obstacle_detection_active != enable:
            self.obstacle_detection_active = enable
            
            # Obstacle wait node'una kontrol mesajı gönder
            control_msg = Bool()
            control_msg.data = enable
            self.obstacle_control_publisher.publish(control_msg)
            
            status = "AKTİF" if enable else "PASİF"
            task_info = ""
            if enable:
                # Hangi görev için aktif olduğunu belirt
                if hasattr(self, 'gorev_listesi') and self.aktif_gorev_index < len(self.gorev_listesi):
                    current_task = self.gorev_listesi[self.aktif_gorev_index]
                    task_info = f" ({current_task['isim']})"
            
            self.get_logger().info(f"🚧 Engel algılama: {status}{task_info}")

    def durum_makinesi_callback(self):
        current_time = time.time()

        # Başlangıç konumu tanımlama durumu
        if self.durum == Durum.BASLANGIC_KONUMU_TANIMLAMA:
            # Sadece başlangıç konumu tanımlanmasını bekle
            return

        # Şarj istasyonu tanımlama durumu
        elif self.durum == Durum.SARJ_ISTASYONU_TANIMLAMA:
            # Sadece şarj istasyonu tanımlanmasını bekle
            return

        elif self.durum == Durum.HEDEF_TANIMLAMA or self.durum == Durum.BOS_BEKLEME:
            return

        elif self.durum == Durum.NAVIGASYONU_BASLAT:
            self.get_logger().info("🚀 Görev dizisi oluşturuluyor ve başlatılıyor...")
            self.navigator.waitUntilNav2Active()

            # Ana görev listesini oluştur
            for i in range(self.total_goals):
                hedef = self.hedefler[i]
                goal_number = i + 1
                gorev_tipi = self.get_task_type_for_goal(goal_number)
                gorev_adi = self.get_task_description(gorev_tipi)
                
                # Ek hareket bilgisini belirle
                if gorev_tipi == GorevTipi.YONLENME:
                    ek_hareket = None  # Yönlenme görevi için ek hareket yok
                elif gorev_tipi == GorevTipi.KUTU_ALMA:
                    ek_hareket = self.forward_speed
                else:  # KUTU_BIRAKMA
                    ek_hareket = self.backward_speed

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
            
            # Görev tipine göre engel algılamayı ayarla
            if gorev['gorev_tipi'] == GorevTipi.YONLENME:
                # Yönlenme görevi → Engel algılamayı aktif et
                self.set_obstacle_detection(True)
                self.get_logger().info(f"🚧 Yönlenme görevi → Engel algılama AKTİF")
            else:
                # Kutu alma/bırakma görevi → Engel algılamayı pasif et
                self.set_obstacle_detection(False)
                self.get_logger().info(f"🚧 Kutu işlemi → Engel algılama PASİF")
            
            self.get_logger().info(f"🎯 Görev {self.aktif_gorev_index + 1}/{len(self.gorev_listesi)}: {gorev['isim']}'e gidiliyor...")
            self.publish_task_status(f"NAVIGATING - {gorev['isim']}")
            self.navigator.goToPose(gorev['hedef_pose'])
            self.durum = Durum.HEDEF_KONTROL

        elif self.durum == Durum.HEDEF_KONTROL:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("✅ Hedefe başarıyla ulaşıldı.")
                    self.current_retry_count = 0  # Başarılı olunca retry sayacını sıfırla
                    
                    # Hedefe ulaştıktan sonra engel algılamayı kapat
                    self.set_obstacle_detection(False)
                    
                    gorev = self.gorev_listesi[self.aktif_gorev_index]
                    
                    # Görev tipine göre farklı davranış
                    if gorev['gorev_tipi'] == GorevTipi.YONLENME:
                        # Yönlenme görevi → Sadece bekle
                        self.get_logger().info(f"📍 YÖNLENME görevi - {self.navigation_wait} saniye bekleme başlıyor...")
                        self.navigation_wait_start_time = current_time
                        self.publish_task_status(f"NAVIGATION_TASK - Waiting {self.navigation_wait}s at position")
                        self.durum = Durum.YONLENME_BEKLEME
                    else:
                        # Kutu alma/bırakma görevi → Ek hareket yap
                        self.durum = Durum.EK_HAREKET_BASLAT
                else:
                    # Navigasyon başarısız - retry mekanizması
                    self.current_retry_count += 1
                    
                    if self.current_retry_count <= self.max_retries:
                        gorev = self.gorev_listesi[self.aktif_gorev_index]
                        self.get_logger().warn(f"❌ Hedefe gidilemedi (Deneme {self.current_retry_count}/{self.max_retries}). Tekrar deneniyor...")
                        self.publish_task_status(f"RETRYING - Attempt {self.current_retry_count}/{self.max_retries}")
                        
                        # Aynı hedefe tekrar git
                        self.navigator.goToPose(gorev['hedef_pose'])
                    else:
                        # Max retry sayısına ulaşıldı
                        self.get_logger().error(f"❌ Hedef {self.max_retries} deneme sonunda başarısız. Sonraki hedefe geçiliyor...")
                        self.current_retry_count = 0
                        self.set_obstacle_detection(False)
                        self.sonraki_goreve_gec()

        elif self.durum == Durum.YONLENME_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Yönlenme bekleme süresi doldu mu kontrol et
            if current_time - self.navigation_wait_start_time >= self.navigation_wait:
                self.get_logger().info('✅ Yönlenme görevi tamamlandı!')
                self.publish_task_status("NAVIGATION_COMPLETED - Moving to next goal")
                self.sonraki_goreve_gec()

        elif self.durum == Durum.EK_HAREKET_BASLAT:
            gorev = self.gorev_listesi[self.aktif_gorev_index]
            if gorev['ek_hareket'] is not None and self.current_pose is not None:
                self.get_logger().info(f"⏱️ {self.task_delay} saniye gecikme başlıyor...")
                self.delay_start_time = current_time
                self.current_task = gorev['gorev_tipi']
                self.publish_task_status(f"TASK_DELAY - Waiting {self.task_delay}s before task")
                self.durum = Durum.EK_HAREKET_GECIKME
            else:
               # Ek hareket yoksa (bu durumda olmamalı) bir sonraki göreve geç
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

        # Servo öncesi bekleme durumu
        elif self.durum == Durum.SERVO_ONCESI_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Servo öncesi bekleme süresi doldu mu kontrol et
            if current_time - self.servo_wait_start_time >= self.pre_servo_wait:
                self.get_logger().info('⏰ Servo öncesi bekleme tamamlandı!')
                
                # Servo kontrolü aktifse servo tetikle, değilse atla
                if self.enable_servo_control:
                    self.trigger_servo()
                    self.durum = Durum.SERVO_TETIKLEME
                else:
                    self.get_logger().info('🤖 Servo kontrolü devre dışı, atlıyor...')
                    # Ses kontrolüne geç
                    if self.enable_sound_control:
                        self.get_logger().info(f"🔊 {self.pre_sound_wait} saniye ses öncesi bekleme başlıyor...")
                        self.sound_wait_start_time = current_time
                        self.publish_task_status(f"PRE_SOUND_WAIT - Waiting {self.pre_sound_wait}s before sound")
                        self.durum = Durum.SES_ONCESI_BEKLEME
                    else:
                        self.get_logger().info(f"⏳ {self.post_task_wait} saniye final bekleme başlıyor...")
                        self.wait_start_time = current_time
                        self.publish_task_status(f"FINAL_TASK_WAIT - Waiting {self.post_task_wait}s")
                        self.durum = Durum.GOREV_SONRASI_BEKLEME

        # Servo tetikleme durumu (sadece async response bekleme)
        elif self.durum == Durum.SERVO_TETIKLEME:
            # Bu durum servo response callback'i ile değiştirilecek
            # Burada sadece timeout kontrolü yapabiliriz (opsiyonel)
            pass

        # Servo sonrası bekleme durumu
        elif self.durum == Durum.SERVO_SONRASI_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Servo sonrası bekleme süresi doldu mu kontrol et
            if current_time - self.servo_wait_start_time >= self.post_servo_wait:
                self.get_logger().info('⏰ Servo sonrası bekleme tamamlandı!')
                
                # Ses kontrolüne geç
                if self.enable_sound_control:
                    self.get_logger().info(f"🔊 {self.pre_sound_wait} saniye ses öncesi bekleme başlıyor...")
                    self.sound_wait_start_time = current_time
                    self.publish_task_status(f"PRE_SOUND_WAIT - Waiting {self.pre_sound_wait}s before sound")
                    self.durum = Durum.SES_ONCESI_BEKLEME
                else:
                    self.get_logger().info('🔊 Ses kontrolü devre dışı, atlıyor...')
                    # Final beklemeye geç
                    self.get_logger().info(f"⏳ {self.post_task_wait} saniye final bekleme başlıyor...")
                    self.wait_start_time = current_time
                    self.publish_task_status(f"FINAL_TASK_WAIT - Waiting {self.post_task_wait}s")
                    self.durum = Durum.GOREV_SONRASI_BEKLEME

        # Ses öncesi bekleme durumu
        elif self.durum == Durum.SES_ONCESI_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Ses öncesi bekleme süresi doldu mu kontrol et
            if current_time - self.sound_wait_start_time >= self.pre_sound_wait:
                self.get_logger().info('⏰ Ses öncesi bekleme tamamlandı!')
                
                # Ses çal ve sonrası beklemeye geç
                self.play_task_sound(self.completed_task_type)
                
                self.get_logger().info(f"🔊 {self.post_sound_wait} saniye ses sonrası bekleme başlıyor...")
                self.sound_wait_start_time = current_time
                self.publish_task_status(f"POST_SOUND_WAIT - Waiting {self.post_sound_wait}s after sound")
                self.durum = Durum.SES_SONRASI_BEKLEME

        # Ses sonrası bekleme durumu
        elif self.durum == Durum.SES_SONRASI_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Ses sonrası bekleme süresi doldu mu kontrol et
            if current_time - self.sound_wait_start_time >= self.post_sound_wait:
                self.get_logger().info('⏰ Ses sonrası bekleme tamamlandı!')
                
                # Final beklemeye geç
                self.get_logger().info(f"⏳ {self.post_task_wait} saniye final bekleme başlıyor...")
                self.wait_start_time = current_time
                self.publish_task_status(f"FINAL_TASK_WAIT - Waiting {self.post_task_wait}s")
                self.durum = Durum.GOREV_SONRASI_BEKLEME
                    
        elif self.durum == Durum.GOREV_SONRASI_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Bekleme süresi doldu mu kontrol et
            if current_time - self.wait_start_time >= self.post_task_wait:
                self.get_logger().info('⏰ Final görev sonrası bekleme tamamlandı!')
                self.get_logger().info('🚀 Sıradaki hedefe geçiliyor...')
                self.publish_task_status("FINAL_TASK_WAIT_COMPLETED - Moving to next goal")
                self.sonraki_goreve_gec()

        # Şarj istasyonu durumları
        elif self.durum == Durum.SARJ_ISTASYONUNA_GIT:
            if self.sarj_istasyonu_pose is None:
                self.get_logger().error("❌ Şarj istasyonu konumu bulunamadı!")
                self.durum = Durum.HATA
                return

            # Şarj istasyonuna navigasyon için engel algılamayı aktif et
            self.set_obstacle_detection(True)
            self.get_logger().info(f"🚧 Şarj istasyonuna gidiş → Engel algılama AKTİF")

            # Şarj istasyonu pozisyonunu PoseStamped formatına çevir
            sarj_goal = PoseStamped()
            sarj_goal.header.frame_id = 'map'
            sarj_goal.header.stamp = self.get_clock().now().to_msg()
            sarj_goal.pose = self.sarj_istasyonu_pose

            x = self.sarj_istasyonu_pose.position.x
            y = self.sarj_istasyonu_pose.position.y
            self.get_logger().info(f"🔌 Şarj istasyonuna gidiliyor: ({x:.2f}, {y:.2f})")
            self.publish_task_status(f"GOING_TO_CHARGE - Battery: {self.current_battery_percentage:.1f}%")
            self.publish_battery_info(f"Low battery! Going to charging station ({x:.2f}, {y:.2f})")

            # Şarj istasyonuna navigasyon başlat
            self.navigator.goToPose(sarj_goal)
            self.durum = Durum.SARJ_ISTASYONU_KONTROL

        elif self.durum == Durum.SARJ_ISTASYONU_KONTROL:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                # Şarj istasyonuna ulaştıktan sonra engel algılamayı kapat
                self.set_obstacle_detection(False)
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("🔌✅ Şarj istasyonuna başarıyla ulaşıldı!")
                    self.get_logger().info(f"🔋 Şarj bekleme başlıyor: {self.charging_wait_time} saniye...")
                    self.publish_task_status(f"AT_CHARGING_STATION - Waiting {self.charging_wait_time}s")
                    self.publish_battery_info(f"Arrived at charging station. Current: {self.current_battery_percentage:.1f}%")
                    
                    self.sarj_bekleme_start_time = current_time
                    self.durum = Durum.SARJ_BEKLEME
                else:
                    self.get_logger().error(f"❌ Şarj istasyonuna gidilemedi (Durum: {result}).")
                    self.publish_task_status("CHARGING_FAILED - Could not reach charging station")
                    # Başarısız olsa bile eski duruma dön
                    self.restore_previous_state()

        elif self.durum == Durum.SARJ_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Şarj bekleme süresi doldu mu veya batarya yeterli seviyeye ulaştı mı?
            charging_time_elapsed = current_time - self.sarj_bekleme_start_time >= self.charging_wait_time
            battery_sufficient = self.current_battery_percentage >= self.full_battery_level
            
            if charging_time_elapsed or battery_sufficient:
                if battery_sufficient:
                    self.get_logger().info(f'🔋✅ Batarya şarj tamamlandı: {self.current_battery_percentage:.1f}%')
                    self.publish_battery_info(f"Battery charged to {self.current_battery_percentage:.1f}%")
                else:
                    self.get_logger().info(f'⏰ Şarj bekleme süresi tamamlandı: {self.current_battery_percentage:.1f}%')
                    self.publish_battery_info(f"Charging time completed. Current: {self.current_battery_percentage:.1f}%")
                
                self.get_logger().info('🚀 Görevlere geri dönülüyor...')
                self.publish_task_status("CHARGING_COMPLETED - Returning to tasks")
                self.restore_previous_state()

        elif self.durum == Durum.BASLANGIC_KONUMA_DON:
            if self.baslangic_pose is None:
                self.get_logger().error("❌ Başlangıç konumu bulunamadı!")
                self.durum = Durum.HATA
                return

            # Başlangıç konumuna dönerken engel algılamayı aktif et
            self.set_obstacle_detection(True)
            self.get_logger().info(f"🚧 Başlangıç konumuna dönüş → Engel algılama AKTİF")

            # Başlangıç pozisyonunu PoseStamped formatına çevir
            baslangic_goal = PoseStamped()
            baslangic_goal.header.frame_id = 'map'
            baslangic_goal.header.stamp = self.get_clock().now().to_msg()
            baslangic_goal.pose = self.baslangic_pose

            x = self.baslangic_pose.position.x
            y = self.baslangic_pose.position.y
            self.get_logger().info(f"🏠 Manuel tanımlanan başlangıç konumuna gidiliyor: ({x:.2f}, {y:.2f})")
            self.publish_task_status(f"RETURNING_HOME - Going to manual start position ({x:.2f}, {y:.2f})")

            # Başlangıç konumuna navigasyon başlat
            self.navigator.goToPose(baslangic_goal)
            self.durum = Durum.BASLANGIC_KONUMA_DON_KONTROL

        elif self.durum == Durum.BASLANGIC_KONUMA_DON_KONTROL:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                # Başlangıç konumuna ulaştıktan sonra engel algılamayı kapat
                self.set_obstacle_detection(False)
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("🏠✅ Manuel tanımlanan başlangıç konumuna başarıyla döndü!")
                    self.publish_task_status("RETURNED_HOME - Successfully returned to manual start position")
                    # YENİ: Görevler tamamlandı işareti
                    self.gorevler_tamamlandi = True
                    # YENİ: Bekleme ve şarj kontrol durumuna geç
                    self.durum = Durum.BEKLEME_VE_SARJ_KONTROL
                    self.get_logger().info("✅ Tüm görevler tamamlandı! Şimdi sadece batarya kontrolü yapılacak.")
                    self.get_logger().info("🔋 Sistem başlangıç noktasında bekleyip batarya durumunu izleyecek.")
                else:
                    self.get_logger().error(f"❌ Başlangıç konumuna dönülemedi (Durum: {result}).")
                    self.publish_task_status("RETURN_HOME_FAILED - Could not return to start position")
                    self.durum = Durum.HATA

        # YENİ: Bekleme ve şarj kontrol durumu
        elif self.durum == Durum.BEKLEME_VE_SARJ_KONTROL:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Bu durumda sürekli bekle, batarya kontrolü timer ile yapılıyor
            # Herhangi bir aksiyon almaya gerek yok, batarya kontrol timer'ı gerektiğinde şarj durumuna geçirecek
            pass

        elif self.durum == Durum.HATA:
            # Hata durumunda engel algılamayı kapat
            self.set_obstacle_detection(False)
            self.get_logger().info("❌ Görev dizisi bir hatadan dolayı sonlandı.")
            
            # Hata durumunda da sistem sıfırlanmasın, sadece bekle
            self.durum = Durum.BEKLEME_VE_SARJ_KONTROL
            self.gorevler_tamamlandi = True
            self.get_logger().info("🔄 Hata sonrası sistem bekleme moduna geçti.")

    # Önceki duruma dön (şarj istasyonundan sonra)
    def restore_previous_state(self):
        """Şarj istasyonundan sonra önceki duruma dön"""
        if self.sarj_oncesi_durum is not None:
            # YENİ: Eğer görevler tamamlandıysa bekleme durumuna dön
            if self.gorevler_tamamlandi:
                self.get_logger().info("🏠 Şarj tamamlandı, başlangıç noktasında bekleme durumuna dönülüyor.")
                self.durum = Durum.BEKLEME_VE_SARJ_KONTROL
                self.publish_task_status("CHARGING_COMPLETED - Waiting at start position")
            else:
                # Önceki görev indeksini geri yükle
                self.aktif_gorev_index = self.sarj_oncesi_gorev_index
                
                # Duruma göre devam et
                if self.sarj_oncesi_durum in [Durum.HEDEFE_GIT, Durum.HEDEF_KONTROL]:
                    # Navigasyon durumlarından geldiyse, hedefe gitmeyi devam et
                    self.durum = Durum.HEDEFE_GIT
                elif self.sarj_oncesi_durum in [Durum.YONLENME_BEKLEME]:
                    # Yönlenme beklemesinden geldiyse, beklemeyi devam et
                    self.navigation_wait_start_time = time.time()  # Bekleme zamanını sıfırla
                    self.durum = Durum.YONLENME_BEKLEME
                elif self.sarj_oncesi_durum == Durum.GOREV_SONRASI_BEKLEME:
                    # Görev sonrası beklemeden geldiyse, sonraki göreve geç
                    self.sonraki_goreve_gec()
                elif self.sarj_oncesi_durum == Durum.BEKLEME_VE_SARJ_KONTROL:
                    # Bekleme durumundan geldiyse tekrar bekleme durumuna dön
                    self.durum = Durum.BEKLEME_VE_SARJ_KONTROL
                else:
                    # Diğer durumlar için güvenli bir nokta
                    self.durum = Durum.HEDEFE_GIT
                    
            # Önceki durum bilgisini temizle
            self.sarj_oncesi_durum = None
            self.sarj_oncesi_gorev_index = 0
        else:
            # Önceki durum bilgisi yoksa
            if self.gorevler_tamamlandi:
                self.durum = Durum.BEKLEME_VE_SARJ_KONTROL
            else:
                self.durum = Durum.HEDEFE_GIT

    def sonraki_goreve_gec(self):
        self.aktif_gorev_index += 1
        if self.aktif_gorev_index < len(self.gorev_listesi):
            self.durum = Durum.HEDEFE_GIT
        else:
            # Tüm görevler tamamlandı
            if self.return_to_start:
                # Başlangıç konumuna dön
                self.get_logger().info("✅ Tüm görevler tamamlandı! Manuel başlangıç konumuna dönülüyor...")
                self.durum = Durum.BASLANGIC_KONUMA_DON
            else:
                # YENİ: Direkt bekleme ve şarj kontrol durumuna geç
                self.get_logger().info("✅ Tüm görevler tamamlandı!")
                self.gorevler_tamamlandi = True
                self.durum = Durum.BEKLEME_VE_SARJ_KONTROL
                self.get_logger().info("🔋 Sistem şimdi sadece batarya kontrolü yapacak.")

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
            elif gorev['gorev_tipi'] == GorevTipi.KUTU_BIRAKMA:
                task_duration = self.backward_duration
                self.task_cmd_vel.linear.x = self.backward_speed
                task_name = "KUTU BIRAKMA (GERİ GİT)"
            else:
                # Bu durumda olmamalı - yönlenme görevleri için ek hareket yok
                self.get_logger().error("❌ Yönlenme görevi için ek hareket çağrıldı!")
                return
            
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

            # Tamamlanan görev tipini sakla
            self.completed_task_type = self.current_task
            
            # Görev durumunu sıfırla
            self.is_executing_task = False
            self.current_task = None
            self.task_cmd_vel = Twist()

            # Kutu bırakma için servo akışı, tüm görevler için ses akışı
            if self.completed_task_type == GorevTipi.KUTU_BIRAKMA:
                # Kutu bırakma → Servo ve ses kontrolü
                self.get_logger().info('🤖 KUTU BIRAKMA tamamlandı - Servo ve ses kontrol akışı başlıyor...')
                
                if self.enable_servo_control:
                    self.get_logger().info(f"🤖 {self.pre_servo_wait} saniye servo öncesi bekleme başlıyor...")
                    self.servo_wait_start_time = time.time()
                    self.publish_task_status(f"PRE_SERVO_WAIT - Waiting {self.pre_servo_wait}s before servo")
                    self.durum = Durum.SERVO_ONCESI_BEKLEME
                elif self.enable_sound_control:
                    # Servo devre dışıysa direkt ses kontrolüne geç
                    self.get_logger().info('🤖 Servo kontrolü devre dışı, ses kontrolüne geçiliyor...')
                    self.get_logger().info(f"🔊 {self.pre_sound_wait} saniye ses öncesi bekleme başlıyor...")
                    self.sound_wait_start_time = time.time()
                    self.publish_task_status(f"PRE_SOUND_WAIT - Waiting {self.pre_sound_wait}s before sound")
                    self.durum = Durum.SES_ONCESI_BEKLEME
                else:
                    # Her ikisi de devre dışıysa direkt final beklemeye geç
                    self.get_logger().info('🤖🔊 Servo ve ses kontrolü devre dışı, final beklemeye geçiliyor...')
                    self.get_logger().info(f"⏳ {self.post_task_wait} saniye final bekleme başlıyor...")
                    self.wait_start_time = time.time()
                    self.publish_task_status(f"FINAL_TASK_WAIT - Waiting {self.post_task_wait}s")
                    self.durum = Durum.GOREV_SONRASI_BEKLEME
                    
            elif self.completed_task_type == GorevTipi.KUTU_ALMA and self.enable_sound_control:
                # Kutu alma → Sadece ses kontrolü
                self.get_logger().info('📦 KUTU ALMA tamamlandı - Ses kontrol akışı başlıyor...')
                self.get_logger().info(f"🔊 {self.pre_sound_wait} saniye ses öncesi bekleme başlıyor...")
                self.sound_wait_start_time = time.time()
                self.publish_task_status(f"PRE_SOUND_WAIT - Waiting {self.pre_sound_wait}s before sound")
                self.durum = Durum.SES_ONCESI_BEKLEME
            else:
                # Ses kontrolü devre dışı → Normal post-task bekleme
                self.get_logger().info(f'⏳ {self.post_task_wait} saniye final bekleme başlıyor...')
                self.publish_task_status(f"FINAL_TASK_WAIT - Waiting {self.post_task_wait}s")
                self.wait_start_time = time.time()
                self.durum = Durum.GOREV_SONRASI_BEKLEME

    # Servo tetikleme fonksiyonu
    def trigger_servo(self):
        """Servo tetikleme servisi çağır"""
        if not self.servo_service_ready:
            self.get_logger().warn('🚫 Servo servisi hazır değil!')
            # Servo başarısız, ses kontrolüne geç
            self.handle_servo_failure()
            return
            
        try:
            request = Trigger.Request()
            
            # Async call yaparak blocking'i önle
            future = self.servo_client.call_async(request)
            future.add_done_callback(self.handle_servo_response)
            
            self.get_logger().info('🤖 Servo tetikleme isteği gönderildi...')
            self.publish_task_status("SERVO_TRIGGERING - Calling servo service")
            
        except Exception as e:
            self.get_logger().error(f'Servo service call hatası: {e}')
            self.handle_servo_failure()

    def handle_servo_response(self, future):
        """Servo service response'unu handle et"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('✅ Servo başarıyla tetiklendi!')
                self.get_logger().info(f'📝 Servo yanıtı: {response.message}')
                self.get_logger().info('🤖 Servo şimdi 90 dereceye gidip 5 saniye bekleyecek!')
                
                # Servo sonrası bekleme başlat
                self.get_logger().info(f"🤖 {self.post_servo_wait} saniye servo sonrası bekleme başlıyor...")
                self.servo_wait_start_time = time.time()
                self.publish_task_status(f"POST_SERVO_WAIT - Waiting {self.post_servo_wait}s after servo")
                self.durum = Durum.SERVO_SONRASI_BEKLEME
            else:
                self.get_logger().error(f'❌ Servo tetikleme başarısız: {response.message}')
                self.handle_servo_failure()
                
        except Exception as e:
            self.get_logger().error(f'Servo service response hatası: {e}')
            self.handle_servo_failure()

    def handle_servo_failure(self):
        """Servo başarısız olduğunda ses kontrolüne geç"""
        if self.enable_sound_control:
            self.get_logger().info('🔊 Servo başarısız, ses kontrolüne geçiliyor...')
            self.get_logger().info(f"🔊 {self.pre_sound_wait} saniye ses öncesi bekleme başlıyor...")
            self.sound_wait_start_time = time.time()
            self.publish_task_status(f"PRE_SOUND_WAIT - Waiting {self.pre_sound_wait}s before sound")
            self.durum = Durum.SES_ONCESI_BEKLEME
        else:
            self.get_logger().info('🔊 Ses kontrolü de devre dışı, final beklemeye geçiliyor...')
            self.get_logger().info(f"⏳ {self.post_task_wait} saniye final bekleme başlıyor...")
            self.wait_start_time = time.time()
            self.publish_task_status(f"FINAL_TASK_WAIT - Waiting {self.post_task_wait}s")
            self.durum = Durum.GOREV_SONRASI_BEKLEME
            
    def play_task_sound(self, task_type):
        """Görev tipine göre ses çal"""
        if not self.services_ready:
            self.get_logger().warn('🚫 Ses servisleri hazır değil!')
            return

        try:
            request = SetBool.Request()
            request.data = True

            if task_type == GorevTipi.KUTU_ALMA:
                # Kutu alma için ses 2
                future = self.sound2_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, "kutu_alma"))
                self.get_logger().info('🔊 Kutu alma sesi çalınıyor...')
            elif task_type == GorevTipi.KUTU_BIRAKMA:
                # Kutu bırakma için ses 1
                future = self.sound1_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, "kutu_birakma"))
                self.get_logger().info('🔊 Kutu bırakma sesi çalınıyor...')
            else:
                # Yönlenme görevi için ses yok
                self.get_logger().info('🔊 Yönlenme görevi - ses çalınmıyor.')

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

    def publish_battery_info(self, info):
        """Batarya bilgisini yayınla"""
        info_msg = String()
        info_msg.data = info
        self.battery_info_publisher.publish(info_msg)

    def emergency_stop(self):
        """Acil durdurma"""
        with self.lock:
            # Acil durumda engel algılamayı kapat
            self.set_obstacle_detection(False)
            
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
        node.get_logger().info('Gelişmiş Çoklu Görev Yöneticisi durduruldu')
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()