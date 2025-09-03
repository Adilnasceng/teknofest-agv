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
    EK_HAREKET_GECIKME = 7    # Gecikme bekleniyor
    CIZGI_TAKIP_KONTROL = 8   # Çizgi takibi aşaması
    OZEL_HAREKET_BASLAT = 9   # Özel hareket aşaması (ileri/dönüş)
    OZEL_HAREKET_KONTROL = 10 # Özel hareket kontrolü
    GOREV_SONRASI_BEKLEME = 11 # Görev sonrası bekleme
    BASLANGIC_KONUMA_DON = 12  # Başlangıç konumuna dön
    BASLANGIC_KONUMA_DON_KONTROL = 13  # Başlangıç konumu kontrolü
    GOREV_BITTI = 14
    HATA = 15

class GorevTipi(Enum):
    CIZGI_TAKIP = "cizgi_takip"      # Çizgi takibi
    KUTU_ALMA = "kutu_alma"          # Kutu alma (ileri hareket)  
    KUTU_BIRAKMA = "kutu_birakma"    # Kutu bırakma (geri hareket)

class CokluGorevYoneticisi(Node):
    def __init__(self):
        super().__init__('dynamic_goal_task_manager')
        self.navigator = BasicNavigator()

        # Parametreler
        self.declare_parameter('total_goals', 6)
        
        # Çizgi takibi parametreleri (süre bazlı kaldırıldı)
        self.declare_parameter('line_lost_timeout', 3.0)      # Çizgi kaybolma timeout süresi
        self.declare_parameter('line_status_check_rate', 0.1) # Çizgi durumu kontrol frekansı
        
        # Özel hareket parametreleri
        self.declare_parameter('forward_speed', 0.2)        # Kutu alma için ileri hız
        self.declare_parameter('forward_duration', 3.0)     # Kutu alma için ileri süresi
        self.declare_parameter('turn_speed', 0.5)           # Kutu bırakma için dönüş hızı  
        self.declare_parameter('turn_duration', 3.14)       # Kutu bırakma için 180° dönüş süresi (pi saniye)
        
        # Genel parametreler
        self.declare_parameter('task_delay', 2.0)
        self.declare_parameter('post_task_wait', 5.0)
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('debug_mode', True)

        self.total_goals = self.get_parameter('total_goals').value
        
        # Çizgi takibi parametreleri
        self.line_lost_timeout = self.get_parameter('line_lost_timeout').value
        self.line_status_check_rate = self.get_parameter('line_status_check_rate').value
        
        # Özel hareket parametreleri
        self.forward_speed = self.get_parameter('forward_speed').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_duration = self.get_parameter('turn_duration').value
        
        # Genel parametreler
        self.task_delay = self.get_parameter('task_delay').value
        self.post_task_wait = self.get_parameter('post_task_wait').value
        self.return_to_start = self.get_parameter('return_to_start').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Durum ve görev yönetimi değişkenleri
        self.durum = Durum.HEDEF_TANIMLAMA
        self.current_pose = None
        self.baslangic_pose = None
        self.hedefler = []
        self.hedef_tanimlama_asama = 1
        self.gorev_listesi = []
        self.aktif_gorev_index = 0

        # Görev çalıştırma için değişkenler
        self.is_executing_task = False
        self.current_task = None
        self.current_phase = None  # "line_follow" veya "special_move"
        self.task_start_time = 0
        self.task_cmd_vel = Twist()
        
        # Çizgi algılama durumu takip değişkenleri
        self.line_follower_active = False
        self.line_detected = False
        self.line_lost_time = None  # Çizginin kaybolma zamanı
        self.last_line_status_time = 0
        
        # Zaman tabanlı kontroller için
        self.delay_start_time = 0
        self.wait_start_time = 0

        # Thread safety
        self.lock = Lock()

        # Service clients - hem çizgi takibi hem de ses servisleri
        self.line_follower_client = self.create_client(SetBool, 'line_follower_control')
        self.sound1_client = self.create_client(SetBool, 'play_sound_1')  # Kutu alma sesi
        self.sound2_client = self.create_client(SetBool, 'play_sound_2')  # Kutu bırakma sesi
        
        self.line_services_ready = False
        self.sound_services_ready = False

        # Subscriber'lar
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        # Çizgi takibi durumu için subscriber - EN ÖNEMLİ EKLENTİ
        self.line_status_sub = self.create_subscription(
            Bool, 
            '/line_follower_status', 
            self.line_status_callback, 
            10
        )

        # Publishers - Twist Mux uyumlu
        # Özel hareket için direkt /cmd_vel yerine /cmd_vel_teleop kullan (yüksek öncelik)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.task_status_publisher = self.create_publisher(String, '/task_status', 10)
        self.goal_info_publisher = self.create_publisher(String, '/goal_info', 10)

        # Ana durum makinesi döngüsü için timer
        self.timer = self.create_timer(0.1, self.durum_makinesi_callback)
        self.service_check_timer = self.create_timer(2.0, self.check_services)

        self.get_logger().info("Çoklu Görev Yöneticisi başlatıldı (Çizgi Algılaması Bazlı).")
        self.get_logger().info("🔍 YENİ ÖZELLİK: Çizgi takibi süre bazlı değil, çizgi kaybolana kadar devam eder")
        self.get_logger().info("📊 Çizgi kaybolma timeout: {}s".format(self.line_lost_timeout))
        self.get_logger().info("KUTU ALMA: Çizgi takibi + İleri hareket")
        self.get_logger().info("KUTU BIRAKMA: Çizgi takibi + 180° dönüş")
        self.get_logger().info("🔧 Twist Mux kullanımı: Navigation pause/resume kaldırıldı")
        self.get_logger().info(f"İleri hareket: {self.forward_speed} m/s ({self.forward_duration}s)")
        self.get_logger().info(f"180° dönüş: {self.turn_speed} rad/s ({self.turn_duration}s)")
        self.get_logger().info(f"Görev gecikmesi: {self.task_delay}s, Görev sonrası bekleme: {self.post_task_wait}s")
        self.get_logger().info(f"Lütfen RViz üzerinden {self.hedef_tanimlama_asama}. hedefi belirleyin.")
        self.publish_task_status(f"GOAL_DEFINITION - Waiting for goal {self.hedef_tanimlama_asama}/{self.total_goals}")

    def line_status_callback(self, msg):
        """Çizgi takibi durumunu takip et"""
        current_time = time.time()
        self.last_line_status_time = current_time
        
        # Çizgi algılama durumunu güncelle
        previous_line_detected = self.line_detected
        self.line_detected = msg.data
        
        # Çizgi durumu değişimi kontrolü
        if self.line_follower_active:  # Sadece çizgi takibi aktifken kontrol et
            if self.line_detected:
                # Çizgi bulundu
                if not previous_line_detected:
                    self.get_logger().info('🔍✅ Çizgi yeniden bulundu!')
                
                # Çizgi kaybolma zamanını sıfırla
                self.line_lost_time = None
                
            else:
                # Çizgi kaybedildi
                if previous_line_detected:
                    self.get_logger().warn('🔍⚠️ Çizgi kaybedildi - timeout başladı!')
                    self.line_lost_time = current_time
                elif self.line_lost_time is None:
                    # İlk kez çizgi kaybolma durumu
                    self.line_lost_time = current_time

    def check_services(self):
        """Service'lerin hazır olup olmadığını kontrol et"""
        line_ready = self.line_follower_client.service_is_ready()
        sound1_ready = self.sound1_client.service_is_ready()
        sound2_ready = self.sound2_client.service_is_ready()
        
        # Çizgi takibi servisi kontrolü
        if line_ready != self.line_services_ready:
            self.line_services_ready = line_ready
            if self.line_services_ready:
                self.get_logger().info('✅ Çizgi takibi servisi hazır!')
            else:
                self.get_logger().warn('⚠️ Çizgi takibi servisi bağlantısı yok!')
        
        # Ses servisleri kontrolü
        new_sound_status = sound1_ready and sound2_ready
        if new_sound_status != self.sound_services_ready:
            self.sound_services_ready = new_sound_status
            if self.sound_services_ready:
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
            self.get_logger().warn("Sistem hedef tanımlama modunda değil, yeni hedef alınamıyor.")
            return

        self.hedefler.append(msg)
        x, y = msg.pose.position.x, msg.pose.position.y
        
        # Görev tipini belirle
        gorev_tipi = self.get_task_type_for_goal(self.hedef_tanimlama_asama)
        if gorev_tipi == GorevTipi.KUTU_ALMA:
            gorev_adi = "KUTU ALMA (Çizgi takibi + İleri hareket)"
        else:
            gorev_adi = "KUTU BIRAKMA (Çizgi takibi + 180° dönüş)"
        
        self.get_logger().info(f"Hedef {self.hedef_tanimlama_asama} kaydedildi: ({x:.2f}, {y:.2f}) - {gorev_adi}")
        self.publish_goal_info(f"Goal {self.hedef_tanimlama_asama}: ({x:.2f}, {y:.2f}) - {gorev_adi}")
        
        self.hedef_tanimlama_asama += 1

        if self.hedef_tanimlama_asama > self.total_goals:
            self.get_logger().info(f"Tüm {self.total_goals} hedef tanımlandı.")
            self.durum = Durum.NAVIGASYONU_BASLAT
        else:
            self.get_logger().info(f"Lütfen RViz üzerinden {self.hedef_tanimlama_asama}. hedefi belirleyin.")
            self.publish_task_status(f"GOAL_DEFINITION - Waiting for goal {self.hedef_tanimlama_asama}/{self.total_goals}")

    def get_task_type_for_goal(self, goal_number):
        """Hedef numarasına göre görev tipini belirle"""
        if goal_number % 2 == 1:  # Tek sayılı hedefler (1, 3, 5)
            return GorevTipi.KUTU_ALMA
        else:  # Çift sayılı hedefler (2, 4, 6)
            return GorevTipi.KUTU_BIRAKMA

    def is_line_completely_lost(self):
        """Çizginin tamamen kaybolup kaybolmadığını kontrol et"""
        if self.line_lost_time is None:
            return False  # Çizgi kaybolmamış
        
        current_time = time.time()
        elapsed = current_time - self.line_lost_time
        return elapsed >= self.line_lost_timeout

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
                gorev_tipi = self.get_task_type_for_goal(goal_number)
                
                if gorev_tipi == GorevTipi.CIZGI_TAKIP:
                    gorev_adi = "ÇİZGİ TAKİBİ"
                elif gorev_tipi == GorevTipi.KUTU_ALMA:
                    gorev_adi = "KUTU ALMA (İleri Git)"
                elif gorev_tipi == GorevTipi.KUTU_BIRAKMA:
                    gorev_adi = "KUTU BIRAKMA (Geri Git)"
                else:
                    gorev_adi = "BİLİNMEYEN GÖREV"
                
                self.gorev_listesi.append({
                    "hedef_pose": hedef, 
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
            if self.current_pose is not None:
                self.get_logger().info(f"⏱️ {self.task_delay} saniye gecikme başlıyor...")
                self.delay_start_time = current_time
                self.current_task = gorev['gorev_tipi']
                self.publish_task_status(f"TASK_DELAY - Waiting {self.task_delay}s before line following")
                self.durum = Durum.EK_HAREKET_GECIKME
            else:
                # Pozisyon bilgisi yoksa bir sonraki göreve geç
                self.sonraki_goreve_gec()

        elif self.durum == Durum.EK_HAREKET_GECIKME:
            # Gecikme süresi doldu mu kontrol et
            if current_time - self.delay_start_time >= self.task_delay:
                # Twist Mux kullanımı ile navigation pause/resume kaldırıldı
                self.start_line_following()
                self.durum = Durum.CIZGI_TAKIP_KONTROL

        elif self.durum == Durum.CIZGI_TAKIP_KONTROL:
            if self.is_executing_task and self.current_phase == "line_follow":
                # ÖNEMLİ: Çizgi durumuna göre kontrol
                if self.line_follower_active:
                    if self.is_line_completely_lost():
                        # Çizgi tamamen kayboldu, çizgi takibini bitir
                        self.get_logger().info(f'🔍⏰ Çizgi {self.line_lost_timeout}s boyunca bulunamadı - Çizgi takibi bitiyor!')
                        self.finish_line_following()
                        self.durum = Durum.OZEL_HAREKET_BASLAT
                    elif self.debug_mode:
                        # Debug bilgisi - çizgi durumu
                        if self.line_detected:
                            if int(current_time * 2) % 10 == 0:  # Her 5 saniyede bir
                                elapsed = current_time - self.task_start_time
                                self.get_logger().info(f'🔍✅ Çizgi takip ediliyor - Süre: {elapsed:.1f}s')
                        else:
                            if self.line_lost_time:
                                lost_duration = current_time - self.line_lost_time
                                remaining = self.line_lost_timeout - lost_duration
                                if int(lost_duration * 10) % 10 == 0:  # Her saniye
                                    self.get_logger().info(f'🔍⚠️ Çizgi kayıp - Timeout: {remaining:.1f}s')

        elif self.durum == Durum.OZEL_HAREKET_BASLAT:
            self.start_special_movement()
            self.durum = Durum.OZEL_HAREKET_KONTROL

        elif self.durum == Durum.OZEL_HAREKET_KONTROL:
            if self.is_executing_task and self.current_phase == "special_move":
                # Özel hareket kontrolü
                elapsed = current_time - self.task_start_time
                
                if self.current_task == GorevTipi.KUTU_ALMA:
                    # İleri hareket kontrolü
                    self.cmd_vel_publisher.publish(self.task_cmd_vel)
                    if elapsed >= self.forward_duration:
                        self.finish_special_movement()
                    elif self.debug_mode and int(elapsed * 10) % 10 == 0:
                        remaining = self.forward_duration - elapsed
                        self.get_logger().info(f'İleri hareket - Kalan: {remaining:.1f}s')
                        
                elif self.current_task == GorevTipi.KUTU_BIRAKMA:
                    # 180° dönüş kontrolü
                    self.cmd_vel_publisher.publish(self.task_cmd_vel)
                    if elapsed >= self.turn_duration:
                        self.finish_special_movement()
                    elif self.debug_mode and int(elapsed * 10) % 10 == 0:
                        remaining = self.turn_duration - elapsed
                        self.get_logger().info(f'180° dönüş - Kalan: {remaining:.1f}s')

        elif self.durum == Durum.GOREV_SONRASI_BEKLEME:
            # Bekleme süresi doldu mu kontrol et
            if current_time - self.wait_start_time >= self.post_task_wait:
                self.get_logger().info('⏰ Görev sonrası bekleme tamamlandı!')
                self.get_logger().info('🚀 Sıradaki hedefe geçiliyor...')
                self.publish_task_status("POST_TASK_WAIT_COMPLETED - Moving to next goal")
                # Twist Mux ile navigation otomatik devam eder
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
            self.baslangic_pose = None
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

    def start_line_following(self):
        """Çizgi takibi aşamasını başlat"""
        with self.lock:
            if self.is_executing_task:
                self.get_logger().warn('Zaten bir görev çalışıyor!')
                return
            
            if not self.line_services_ready:
                self.get_logger().error('Çizgi takibi servisi hazır değil!')
                return
            
            self.is_executing_task = True
            self.current_phase = "line_follow"
            self.task_start_time = time.time()
            self.line_follower_active = True  # Çizgi takibi aktif oldu
            
            # Çizgi durumu değişkenlerini sıfırla
            self.line_detected = False
            self.line_lost_time = None
            
            task_name = "KUTU ALMA" if self.current_task == GorevTipi.KUTU_ALMA else "KUTU BIRAKMA"
            self.get_logger().info(f'GÖREV BAŞLATILDI: {task_name} - Aşama 1: Çizgi takibi')
            self.get_logger().info(f'🔍 Çizgi kaybolma timeout: {self.line_lost_timeout}s')
            self.get_logger().info('🔧 Twist Mux: Çizgi takibi öncelik kazandı, navigation otomatik pause')
            
            self.publish_task_status(f"LINE_FOLLOW_STARTED - {task_name} - Until line is lost ({self.line_lost_timeout}s timeout)")
            
            # Çizgi takibini başlat - Twist mux otomatik olarak önceliği çizgi takibine verecek
            self.call_line_follower_service(True)

    def finish_line_following(self):
        """Çizgi takibi aşamasını bitir"""
        with self.lock:
            if not self.is_executing_task or self.current_phase != "line_follow":
                return
            
            self.line_follower_active = False  # Çizgi takibi artık aktif değil
            
            # Çizgi takibini durdur - Twist mux otomatik olarak navigation'ı restore edecek
            self.call_line_follower_service(False)
            
            task_name = "KUTU ALMA" if self.current_task == GorevTipi.KUTU_ALMA else "KUTU BIRAKMA"
            elapsed = time.time() - self.task_start_time
            self.get_logger().info(f'Aşama 1 tamamlandı: {task_name} - Çizgi takibi bitti (Süre: {elapsed:.1f}s)')
            self.get_logger().info('🔧 Twist Mux: Navigation restored, özel hareket için teleop aktifleşecek')
            
            self.publish_task_status(f"LINE_FOLLOW_COMPLETED - {task_name} - Moving to special movement (Duration: {elapsed:.1f}s)")

    def start_special_movement(self):
        """Özel hareket aşamasını başlat (İleri hareket veya 180° dönüş)"""
        with self.lock:
            self.current_phase = "special_move"
            self.task_start_time = time.time()
            
            if self.current_task == GorevTipi.KUTU_ALMA:
                # İleri hareket hazırla
                self.task_cmd_vel.linear.x = self.forward_speed
                self.task_cmd_vel.linear.y = 0.0
                self.task_cmd_vel.linear.z = 0.0
                self.task_cmd_vel.angular.x = 0.0
                self.task_cmd_vel.angular.y = 0.0
                self.task_cmd_vel.angular.z = 0.0
                
                task_name = "KUTU ALMA - Aşama 2: İleri hareket"
                duration = self.forward_duration
                self.get_logger().info(f'{task_name}')
                self.get_logger().info(f'Hız: {self.forward_speed} m/s, Süre: {duration}s')
                
                # Ses çal
                self.play_task_sound()
                
            elif self.current_task == GorevTipi.KUTU_BIRAKMA:
                # 180° dönüş hazırla
                self.task_cmd_vel.linear.x = 0.0
                self.task_cmd_vel.linear.y = 0.0
                self.task_cmd_vel.linear.z = 0.0
                self.task_cmd_vel.angular.x = 0.0
                self.task_cmd_vel.angular.y = 0.0
                self.task_cmd_vel.angular.z = self.turn_speed  # Saat yönü tersi dönüş
                
                task_name = "KUTU BIRAKMA - Aşama 2: 180° dönüş"
                duration = self.turn_duration
                self.get_logger().info(f'{task_name}')
                self.get_logger().info(f'Dönüş hızı: {self.turn_speed} rad/s, Süre: {duration}s')
                
                # Ses çal
                self.play_task_sound()
            
            self.get_logger().info('🔧 Twist Mux: Teleop prioritesi aktif (navigation override)')
            self.publish_task_status(f"SPECIAL_MOVEMENT_STARTED - {task_name} - Duration: {duration}s")

    def finish_special_movement(self):
        """Özel hareket aşamasını bitir"""
        with self.lock:
            if not self.is_executing_task or self.current_phase != "special_move":
                return
            
            # Robot durdur - Twist mux teleop priority'yi sonlandır
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
            task_name = "KUTU ALMA" if self.current_task == GorevTipi.KUTU_ALMA else "KUTU BIRAKMA"
            self.get_logger().info(f'Aşama 2 tamamlandı: {task_name} - Özel hareket bitti')
            self.get_logger().info(f'GÖREV TAMAMLANDI: {task_name}')
            self.get_logger().info(f'{self.post_task_wait} saniye bekleme başlıyor...')
            self.get_logger().info('🔧 Twist Mux: Navigation otomatik restore (normal priority)')
            
            self.publish_task_status(f"TASK_COMPLETED - {task_name} - Waiting {self.post_task_wait}s")
            
            # Görev durumunu sıfırla
            self.is_executing_task = False
            self.current_task = None
            self.current_phase = None
            self.task_cmd_vel = Twist()
            
            # Post-task bekleme başlat
            self.wait_start_time = time.time()
            self.durum = Durum.GOREV_SONRASI_BEKLEME

    def call_line_follower_service(self, start_following):
        """Çizgi takibi servisini çağır"""
        if not self.line_services_ready:
            self.get_logger().error('❌ Çizgi takibi servisi hazır değil!')
            return
        
        try:
            request = SetBool.Request()
            request.data = start_following
            
            action_name = "başlatılıyor" if start_following else "durduruluyor"
            self.get_logger().info(f'🔍 Çizgi takibi {action_name}...')
            
            future = self.line_follower_client.call_async(request)
            future.add_done_callback(lambda f: self.line_follower_callback(f, start_following))
                
        except Exception as e:
            self.get_logger().error(f'Çizgi takibi servis çağırma hatası: {e}')

    def line_follower_callback(self, future, start_following):
        """Çizgi takibi servis callback"""
        try:
            response = future.result()
            if response.success:
                action_name = "başlatıldı" if start_following else "durduruldu"
                self.get_logger().info(f'🔍✅ Çizgi takibi {action_name}!')
            else:
                action_name = "başlatma" if start_following else "durdurma"
                self.get_logger().error(f'❌ Çizgi takibi {action_name} başarısız: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Çizgi takibi servis callback hatası: {e}')

    def play_task_sound(self):
        """Görev tipine göre ses çal (sadece kutu alma/bırakma için)"""
        if not self.sound_services_ready:
            return
        
        try:
            request = SetBool.Request()
            request.data = True
            
            if self.current_task == GorevTipi.KUTU_ALMA:
                # Kutu alma için ses 1
                future = self.sound1_client.call_async(request)
                future.add_done_callback(lambda f: self.sound_callback(f, "kutu_alma"))
            elif self.current_task == GorevTipi.KUTU_BIRAKMA:
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
                # Görev aşamasına göre durdurma
                if self.current_phase == "line_follow":
                    self.call_line_follower_service(False)
                    self.line_follower_active = False
                elif self.current_phase == "special_move":
                    stop_cmd = Twist()
                    self.cmd_vel_publisher.publish(stop_cmd)
                
                self.is_executing_task = False
                self.current_task = None
                self.current_phase = None
                self.task_cmd_vel = Twist()
                
                self.get_logger().warn('ACİL DURDURMA - Tüm görevler iptal edildi!')
                self.get_logger().info('🔧 Twist Mux: Acil durdurma sonrası navigation restore')
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