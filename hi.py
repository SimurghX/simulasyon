import asyncio
import threading
import cv2
import numpy as np
from mavsdk import System
from mavsdk.action import ActionError

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RedDetectorNode(Node):
    # ... (Bu kısım önceki kodla tamamen aynı kalacak, buraya eski class içeriğini yapıştırabilirsin)
    def __init__(self):
        super().__init__('red_detector_node')
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.bridge = CvBridge()
        self.relative_x = 0.0
        self.relative_y = 0.0
        self.is_target_detected = False

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Görüntü dönüştürme hatası: {e}")
            return

        height, width, _ = cv_image.shape
        center_x, center_y = width // 2, height // 2

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                self.relative_x = cx - center_x
                self.relative_y = cy - center_y
                self.is_target_detected = True
                
                cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.line(cv_image, (center_x, center_y), (cx, cy), (255, 0, 0), 2)
                cv2.putText(cv_image, f"X:{self.relative_x} Y:{self.relative_y}", (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            self.is_target_detected = False

        cv2.imshow("Red Object Tracking", cv_image)
        cv2.waitKey(1)

def ros_spin_thread(node):
    rclpy.spin(node)

async def run():
    rclpy.init()
    red_detector = RedDetectorNode()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(red_detector,), daemon=True)
    spin_thread.start()

    drone = System()
    
    # try...finally yapısı ile program çökse bile temiz kapanış garantilenir
    try:
        print("Drona bağlanılıyor...")
        # UDP uyarısını gidermek için udpin:// kullanıyoruz
        await drone.connect(system_address="udpin://127.0.0.1:14540")
        
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Drone bağlandı!")
                break
        
        print("Global pozisyon ve arm edilebilirlik bekleniyor (Bu biraz sürebilir)...")
        # SADECE global pozisyon değil, is_armable durumunu da bekliyoruz
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok and health.is_armable:
                print("Tüm uçuş öncesi kontroller tamamlandı. Drone hazır.")
                break
        
        print("Arm ediliyor...")
        await drone.action.arm()
        print("Arm başarılı!")
        
        await drone.action.set_takeoff_altitude(5.0)
        
        print("Takeoff yapılıyor...")
        await drone.action.takeoff()
        print("Takeoff başarılı!")
        
        print("Havada bekleniyor (10 saniye)...")
        for _ in range(100000000000000):
            if red_detector.is_target_detected:
                print(f"[HEDEF GÖRÜLDÜ] X={red_detector.relative_x}px, Y={red_detector.relative_y}px")
            else:
                print("[HEDEF ARANIYOR...] Kırmızı nesne kadrajda değil.")
            await asyncio.sleep(1)
        
        print("İniş yapılıyor...")
        await drone.action.land()
        
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("İniş tamamlandı!")
                break
            await asyncio.sleep(1)

    except ActionError as e:
        print(f"\n[HATA] Drone eylemi gerçekleştirilemedi: {e}")
    except Exception as e:
        print(f"\n[HATA] Beklenmeyen bir durum oluştu: {e}")
    finally:
        # Kod normal bitse de, hata alıp catch bloğuna girse de burası çalışır.
        print("\nSistem temizleniyor ve kapatılıyor...")
        red_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(run())