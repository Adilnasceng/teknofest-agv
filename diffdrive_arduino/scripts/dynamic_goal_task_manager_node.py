#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
import tf_transformations
import math
import time
from threading import Lock
from enum import Enum

# Programın durumlarını tanımlayan Enum sınıfı
class Durum(Enum):
    BASLANGIC_KONUMU_TANIMLAMA = 0  # YENİ: Manuel başlangıç konumu tanımlama
    BOS_BEKLEME = 1
    HEDEF_TANIMLAMA = 2
    NAVIGASYONU_BASLAT = 3
    HEDEFE_GIT = 4
    HEDEF_KONTROL = 5
    EK_HAREKET_BASLAT = 6
    EK_HAREKET_GECIKME = 7    # Gecikme bekleniyor
    EK_HAREKET_KONTROL = 8
    YONLENME_BEKLEME = 9     # YENİ: Yönlenme görevi bekleme
    # YENİ: Servo ve ses kontrolü için durumlar
    SERVO_ONCESI_BEKLEME = 10     # Servo tetiklemeden önce bekleme
    SERVO_TETIKLEME = 11          # Servo tetikleme
    SERVO_SONRASI_BEKLEME = 12    # Servo tetikleme sonrası bekleme
    SES_ONCESI_BEKLEME = 13       # Ses çalmadan önce bekleme
    SES_SONRASI_BEKLEME = 14      # Ses sonrası bekleme
    GOREV_SONRASI_BEKLEME = 15    # Final görev sonrası bekleme
    BASLANGIC_KONUMA_DON = 16     # Başlangıç konumuna dön
    BASLANGIC_KONUMA_DON_KONTROL = 17  # Başlangıç konumu kontrolü
    GOREV_BITTI = 18
    HATA = 19

class GorevTipi(Enum):
    YONLENME = "yonlenme"        # YENİ: Sadece hedefe git ve bekle
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
        self.declare_parameter('navigation_wait', 5.0)  # YENİ: Yönlenme bekleme süresi
        
        # YENİ: Servo ve ses kontrol parametreleri
        self.declare_parameter('pre_servo_wait', 5.0)     # Servo tetiklemeden önce bekleme
        self.declare_parameter('post_servo_wait', 5.0)    # Servo tetikleme sonrası bekleme  
        self.declare_parameter('pre_sound_wait', 5.0)     # Ses çalmadan önce bekleme
        self.declare_parameter('post_sound_wait', 5.0)    # Ses sonrası bekleme
        self.declare_parameter('post_task_wait', 5.0)     # Final görev sonrası bekleme süresi
        self.declare_parameter('enable_servo_control', True)  # Servo kontrol aktif/pasif
        self.declare_parameter('enable_sound_control', True)  # Ses kontrol aktif/pasif
        
        self.declare_parameter('return_to_start', True) # Başlangıça dönüş
        self.declare_parameter('debug_mode', True)
        # YENİ: Engel algılama kontrolü parametreleri
        self.declare_parameter('enable_obstacle_control', True)  # Engel algılama kontrolü aktif/pasif
        
        self.base_goals = self.get_parameter('base_goals').value
        self.total_goals = self.base_goals * 2  # YENİ: Her temel görev için 2 hedef
        self.forward_speed = self.get_parameter('forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.backward_duration = self.get_parameter('backward_duration').value
        self.task_delay = self.get_parameter('task_delay').value
        self.navigation_wait = self.get_parameter('navigation_wait').value  # YENİ
        
        # YENİ: Servo ve ses kontrol parametreleri
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

        # Durum ve görev yönetimi değişkenleri
        self.durum = Durum.BASLANGIC_KONUMU_TANIMLAMA  # YENİ: Başlangıç konumu tanımlama ile başla
        self.current_pose = None
        self.baslangic_pose = None  # Manuel olarak tanımlanacak başlangıç pozisyonu
        self.baslangic_konumu_tanimlandi = False  # YENİ: Başlangıç konumunun tanımlandığını takip et
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
        self.navigation_wait_start_time = 0  # YENİ: Yönlenme bekleme zamanı
        
        # YENİ: Servo ve ses kontrol zamanları
        self.servo_wait_start_time = 0
        self.sound_wait_start_time = 0

        # YENİ: Engel algılama kontrolü durumu
        self.obstacle_detection_active = False

        # Thread safety
        self.lock = Lock()
        
        # Service clients - ses ve servo için
        self.sound1_client = self.create_client(SetBool, 'play_sound_1')
        self.sound2_client = self.create_client(SetBool, 'play_sound_2')
        # YENİ: Servo kontrol client
        self.servo_client = self.create_client(Trigger, '/trigger_servo')
        
        self.services_ready = False
        self.servo_service_ready = False  # YENİ: Servo service durumu
        
        # Subscriber'lar
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_status_publisher = self.create_publisher(String, '/task_status', 10)
        self.goal_info_publisher = self.create_publisher(String, '/goal_info', 10)
        
        # YENİ: Engel algılama kontrolü için publisher
        self.obstacle_control_publisher = self.create_publisher(Bool, '/obstacle_wait_enable', 10)

        # Ana durum makinesi döngüsü için timer
        self.timer = self.create_timer(0.1, self.durum_makinesi_callback)
        self.service_check_timer = self.create_timer(2.0, self.check_services)

        self.get_logger().info("🎯 Gelişmiş Çoklu Görev Yöneticisi başlatıldı.")
        self.get_logger().info("📋 YENİ Görev sistemi: Yönlenme → Kutu Alma → Yönlenme → Kutu Bırakma")
        self.get_logger().info(f"🔢 {self.base_goals} temel görev = {self.total_goals} toplam hedef")
        self.get_logger().info(f"⚡ İleri: {self.forward_speed} m/s ({self.forward_duration}s), Geri: {self.backward_speed} m/s ({self.backward_duration}s)")
        self.get_logger().info(f"⏱️ Yönlenme bekleme: {self.navigation_wait}s, Görev gecikmesi: {self.task_delay}s")
        
        # YENİ: Servo ve ses kontrol log'ları
        if self.enable_servo_control:
            self.get_logger().info(f"🤖 Servo kontrolü AKTİF - Öncesi: {self.pre_servo_wait}s, Sonrası: {self.post_servo_wait}s")
        else:
            self.get_logger().info("🤖 Servo kontrolü PASİF")
            
        if self.enable_sound_control:
            self.get_logger().info(f"🔊 Ses kontrolü AKTİF - Öncesi: {self.pre_sound_wait}s, Sonrası: {self.post_sound_wait}s")
        else:
            self.get_logger().info("🔊 Ses kontrolü PASİF")
            
        self.get_logger().info(f"⏰ Final görev sonrası bekleme: {self.post_task_wait}s")
        
        # YENİ: Engel algılama kontrolü log'u
        if self.enable_obstacle_control:
            self.get_logger().info("🚧 Engel algılama kontrolü AKTİF - Yönlenme görevlerinde çalışacak")
        else:
            self.get_logger().info("🚧 Engel algılama kontrolü PASİF")
            
        # YENİ: İlk olarak başlangıç konumu tanımlama talimatı
        self.get_logger().info("🏠 ÖNCE RViz üzerinden BAŞLANGIÇ KONUMUNU belirleyin.")
        self.get_logger().info("📍 Başlangıç konumu tanımlandıktan sonra hedefler tanımlanacak.")
        self.publish_task_status("START_POSITION_DEFINITION - Set start position first")

        # Başlangıçta engel algılamayı kapat
        self.set_obstacle_detection(False)

    def check_services(self):
        """Service'lerin hazır olup olmadığını kontrol et"""
        sound1_ready = self.sound1_client.service_is_ready()
        sound2_ready = self.sound2_client.service_is_ready()
        servo_ready = self.servo_client.service_is_ready()  # YENİ: Servo service kontrolü

        new_sound_status = sound1_ready and sound2_ready
        if new_sound_status != self.services_ready:
            self.services_ready = new_sound_status
            if self.services_ready:
                self.get_logger().info('✅ Ses servisleri hazır!')
            else:
                self.get_logger().warn('⚠️ Ses servisleri bağlantısı yok!')
                
        # YENİ: Servo service durumu
        if servo_ready != self.servo_service_ready:
            self.servo_service_ready = servo_ready
            if self.servo_service_ready:
                self.get_logger().info('✅ Servo servisi hazır!')
            else:
                self.get_logger().warn('⚠️ Servo servisi bağlantısı yok!')

    def pose_callback(self, msg):
        # YENİ: Sadece mevcut pozisyonu güncelle, otomatik başlangıç kaydı yapma
        self.current_pose = msg.pose.pose

    def goal_pose_callback(self, msg):
        # YENİ: Önce başlangıç konumu tanımlama kontrolü
        if self.durum == Durum.BASLANGIC_KONUMU_TANIMLAMA:
            if not self.baslangic_konumu_tanimlandi:
                # Başlangıç konumunu manuel olarak tanımla
                self.baslangic_pose = msg.pose
                self.baslangic_konumu_tanimlandi = True
                
                x, y = msg.pose.position.x, msg.pose.position.y
                self.get_logger().info(f"🏠✅ Başlangıç konumu manuel olarak tanımlandı: ({x:.2f}, {y:.2f})")
                self.publish_goal_info(f"Start Position: ({x:.2f}, {y:.2f})")
                
                # Hedef tanımlama durumuna geç
                self.durum = Durum.HEDEF_TANIMLAMA
                
                # İlk hedef tanımlama talimatı
                next_task_type = self.get_task_type_for_goal(self.hedef_tanimlama_asama)
                next_task_desc = self.get_task_description(next_task_type)
                self.get_logger().info(f"🎯 Şimdi RViz üzerinden {self.hedef_tanimlama_asama}. hedefi belirleyin ({next_task_desc}).")
                self.publish_task_status(f"GOAL_DEFINITION - Waiting for goal {self.hedef_tanimlama_asama}/{self.total_goals} ({next_task_desc})")
                return
            else:
                self.get_logger().warn("❌ Başlangıç konumu zaten tanımlandı.")
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
        """YENİ: Engel algılama durumunu ayarla"""
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

        # YENİ: Başlangıç konumu tanımlama durumu
        if self.durum == Durum.BASLANGIC_KONUMU_TANIMLAMA:
            # Sadece başlangıç konumu tanımlanmasını bekle
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
            
            # YENİ: Görev tipine göre engel algılamayı ayarla
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
                    
                    # YENİ: Hedefe ulaştıktan sonra engel algılamayı kapat
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
                    self.get_logger().error(f"❌ Hedefe gidilemedi (Durum: {result}).")
                    # Hata durumunda da engel algılamayı kapat
                    self.set_obstacle_detection(False)
                    self.durum = Durum.HATA

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

        # YENİ: Servo öncesi bekleme durumu
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

        # YENİ: Servo tetikleme durumu (sadece async response bekleme)
        elif self.durum == Durum.SERVO_TETIKLEME:
            # Bu durum servo response callback'i ile değiştirilecek
            # Burada sadece timeout kontrolü yapabiliriz (opsiyonel)
            pass

        # YENİ: Servo sonrası bekleme durumu
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

        # YENİ: Ses öncesi bekleme durumu
        elif self.durum == Durum.SES_ONCESI_BEKLEME:
            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            # Ses öncesi bekleme süresi doldu mu kontrol et
            if current_time - self.sound_wait_start_time >= self.pre_sound_wait:
                self.get_logger().info('⏰ Ses öncesi bekleme tamamlandı!')
                
                # Ses çal ve sonrası beklemeye geç
                self.play_task_sound()
                
                self.get_logger().info(f"🔊 {self.post_sound_wait} saniye ses sonrası bekleme başlıyor...")
                self.sound_wait_start_time = current_time
                self.publish_task_status(f"POST_SOUND_WAIT - Waiting {self.post_sound_wait}s after sound")
                self.durum = Durum.SES_SONRASI_BEKLEME

        # YENİ: Ses sonrası bekleme durumu
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

        elif self.durum == Durum.BASLANGIC_KONUMA_DON:
            if self.baslangic_pose is None:
                self.get_logger().error("❌ Başlangıç konumu bulunamadı!")
                self.durum = Durum.HATA
                return

            # YENİ: Başlangıç konumuna dönerken engel algılamayı aktif et
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
                # YENİ: Başlangıç konumuna ulaştıktan sonra engel algılamayı kapat
                self.set_obstacle_detection(False)
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("🏠✅ Manuel tanımlanan başlangıç konumuna başarıyla döndü!")
                    self.publish_task_status("RETURNED_HOME - Successfully returned to manual start position")
                    self.durum = Durum.GOREV_BITTI
                else:
                    self.get_logger().error(f"❌ Başlangıç konumuna dönülemedi (Durum: {result}).")
                    self.publish_task_status("RETURN_HOME_FAILED - Could not return to start position")
                    self.durum = Durum.HATA

        elif self.durum == Durum.GOREV_BITTI or self.durum == Durum.HATA:
            # YENİ: Son durumda engel algılamayı kapat
            self.set_obstacle_detection(False)
            
            if self.durum == Durum.GOREV_BITTI: 
                self.get_logger().info("🎉 Tüm görevler başarıyla tamamlandı ve manuel başlangıç konumuna döndü!")
            else: 
                self.get_logger().info("❌ Görev dizisi bir hatadan dolayı sonlandı.")

            # Sistemi sıfırla - YENİ: Başlangıç konumu tanımlama ile başla
            self.hedefler = []
            self.gorev_listesi = []
            self.hedef_tanimlama_asama = 1
            self.aktif_gorev_index = 0
            self.baslangic_pose = None  # Başlangıç pozisyonunu sıfırla
            self.baslangic_konumu_tanimlandi = False  # YENİ: Başlangıç konumu sıfırla
            self.nav2_ready = False  # Navigator kontrolünü sıfırla
            self.durum = Durum.BASLANGIC_KONUMU_TANIMLAMA  # YENİ: Başlangıç konumu tanımlama ile başla
            self.get_logger().info("🔄 Sistem yeni görevler için hazır.")
            self.get_logger().info("🏠 ÖNCE RViz üzerinden YENİ BAŞLANGIÇ KONUMUNU belirleyin.")
            self.publish_task_status("SYSTEM_RESET - Set new start position first")

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
        """Görev çalıştırma bitir - YENİ: Servo ve ses kontrol akışı"""
        with self.lock:
            if not self.is_executing_task:
                return

            # Robot durdur
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

            task_name = "KUTU ALMA" if self.current_task == GorevTipi.KUTU_ALMA else "KUTU BIRAKMA"
            self.get_logger().info(f'✅ GÖREV TAMAMLANDI: {task_name}')

            # Görev durumunu sıfırla
            self.is_executing_task = False
            current_task_type = self.current_task  # Görev tipini sakla
            self.current_task = None
            self.task_cmd_vel = Twist()

            # YENİ: Kutu bırakma göreviyse servo ve ses kontrol akışını başlat
            if current_task_type == GorevTipi.KUTU_BIRAKMA:
                self.get_logger().info('🤖 KUTU BIRAKMA tamamlandı - Servo ve ses kontrol akışı başlıyor...')
                
                # Servo kontrolü aktifse servo bekleme başlat
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
            else:
                # Kutu alma veya yönlenme görevi → Normal post-task bekleme
                self.get_logger().info(f'⏳ {self.post_task_wait} saniye final bekleme başlıyor...')
                self.publish_task_status(f"FINAL_TASK_WAIT - Waiting {self.post_task_wait}s")
                self.wait_start_time = time.time()
                self.durum = Durum.GOREV_SONRASI_BEKLEME

    # YENİ: Servo tetikleme fonksiyonu
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
            
    def play_task_sound(self):
        """Görev tipine göre ses çal"""
        if not self.services_ready:
            self.get_logger().warn('🚫 Ses servisleri hazır değil!')
            return

        try:
            request = SetBool.Request()
            request.data = True

            if self.current_task == GorevTipi.KUTU_ALMA:
                # Kutu alma için ses 1
                future = self.sound1_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, "kutu_alma"))
                self.get_logger().info('🔊 Kutu alma sesi çalınıyor...')
            elif self.current_task == GorevTipi.KUTU_BIRAKMA:
                # Kutu bırakma için ses 2
                future = self.sound2_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, "kutu_birakma"))
                self.get_logger().info('🔊 Kutu bırakma sesi çalınıyor...')
            else:
                # Genel durum için kutu bırakma sesi (çünkü bu akış kutu bırakma sonrası)
                future = self.sound2_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, "kutu_birakma"))
                self.get_logger().info('🔊 Görev tamamlama sesi çalınıyor...')

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
            # YENİ: Acil durumda engel algılamayı kapat
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