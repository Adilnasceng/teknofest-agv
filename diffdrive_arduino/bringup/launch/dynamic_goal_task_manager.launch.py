#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Enhanced Task Manager Node - servo ve ses kontrollü görev yöneticisi
    enhanced_task_manager_node = Node(
        package='diffdrive_arduino',
        executable='dynamic_goal_task_manager_node.py',
        name='enhanced_task_manager',
        output='screen',
        parameters=[{
            # Temel görev parametreleri
            'base_goals': 2,                    # 6 temel görev = 12 toplam hedef
            'forward_speed': 0.2,               # İleri hareket hızı (m/s)
            'backward_speed': -0.2,             # Geri hareket hızı (m/s)
            'forward_duration': 3.0,            # İleri hareket süresi (s)
            'backward_duration': 3.0,           # Geri hareket süresi (s)
            'task_delay': 2.0,                  # Ek hareket başlamadan önce gecikme (s)
            'navigation_wait': 5.0,             # Yönlenme görevi bekleme süresi (s)
            
            # YENİ: Servo kontrol parametreleri
            'enable_servo_control': True,       # Servo kontrolü aktif/pasif
            'pre_servo_wait': 5.0,              # Servo tetiklemeden önce bekleme (s)
            'post_servo_wait': 5.0,             # Servo tetikleme sonrası bekleme (s)
            
            # YENİ: Ses kontrol parametreleri  
            'enable_sound_control': True,       # Ses kontrolü aktif/pasif
            'pre_sound_wait': 5.0,              # Ses çalmadan önce bekleme (s)
            'post_sound_wait': 5.0,             # Ses sonrası bekleme (s)
            
            # Final parametreler
            'post_task_wait': 5.0,              # Final görev sonrası bekleme (s)
            'return_to_start': True,            # Başlangıç konumuna dönüş
            'debug_mode': True,                 # Debug modu
            
            # Engel algılama kontrolü
            'enable_obstacle_control': True,    # Engel algılama kontrolü aktif/pasif
        }]
    )

    return LaunchDescription([
        enhanced_task_manager_node
    ])



# KULLANIM TALİMATLARI:
# 
# 1. Node'u başlat:
#    ros2 launch diffdrive_arduino dynamic_goal_task_manager.launch.py
#
# 2. RViz'de önce başlangıç konumunu belirle (2D Pose Estimate)
#
# 3. RViz'de hedefleri sırayla belirle (2D Nav Goal):
#    - Hedef 1: YÖNLENME (Git ve Bekle)
#    - Hedef 2: KUTU ALMA (İleri Git)
#    - Hedef 3: YÖNLENME (Git ve Bekle)  
#    - Hedef 4: KUTU BIRAKMA (Geri Git + Servo + Ses)
#
# 4. KUTU BIRAKMA görevlerinde akış:
#    Robot hedefe gider → Geri hareket yapar → 5s bekler →
#    Servo tetiklenir → 5s bekler → Ses çalar → 5s bekler →
#    Sonraki göreve geçer
#
# PARAMETRE AÇIKLAMALARI:
# 
# pre_servo_wait: Ek hareket bittikten sonra servo tetiklenmeden önce bekleme
# post_servo_wait: Servo tetiklendikten sonra bekleme  
# pre_sound_wait: Ses çalmadan önce bekleme
# post_sound_wait: Ses çaldıktan sonra bekleme
# post_task_wait: Final bekleme (hepsi bittikten sonra)
#
# enable_servo_control: false yaparsanız servo atlanır
# enable_sound_control: false yaparsanız ses atlanır
#
# SERVİS GEREKLİLİKLERİ:
# - /trigger_servo (std_srvs/srv/Trigger): Servo kontrolü için
# - /play_sound_1 (std_srvs/srv/SetBool): Kutu alma sesi
# - /play_sound_2 (std_srvs/srv/SetBool): Kutu bırakma sesi