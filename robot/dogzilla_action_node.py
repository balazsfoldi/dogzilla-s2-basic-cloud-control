import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import random

# --- IMPORT BEÁLLÍTÁSOK ---
sys.path.append('/home/pi/DOGZILLA/DOGZILLALib')

try:
    from DOGZILLALib.DOGZILLALib import DOGZILLA
    HARDWARE_AVAILABLE = True
except ImportError as e:
    print(f"HIBA: {e}")
    HARDWARE_AVAILABLE = False
    class DOGZILLA:
        def action(self, code): print(f"[SIM] Action Code: {code}")
        def move(self, x, y, r): print(f"[SIM] Move: x={x}, y={y}, r={r}")
        def forward(self, speed): print(f"[SIM] Forward: {speed}")
        def back(self, speed): print(f"[SIM] Back: {speed}")
        def left(self, speed): print(f"[SIM] Left: {speed}")
        def right(self, speed): print(f"[SIM] Right: {speed}")
        def turnleft(self, speed): print(f"[SIM] Turn Left: {speed}")
        def turnright(self, speed): print(f"[SIM] Turn Right: {speed}")
        def stop(self): print("[SIM] STOP")
        def read_battery(self): return 8.2

class DogzillaAllInOneNode(Node):
    def __init__(self):
        super().__init__('dogzilla_action_node')
        
        self.subscription = self.create_subscription(
            String,
            'robot/cmd',
            self.listener_callback,
            10)

        # Uplink (Telemetria küldés) - Erre a topikra küldjük az adatokat
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        
        # Időzítő: 2 másodpercenként küldjön adatot (0.5 Hz)
        self.timer = self.create_timer(2.0, self.timer_callback)

        # TRÜKKÖK LISTÁJA (1-20)
        self.action_map = {
            "lie_down": 1, "stand_up": 2, "crawl": 3, "turn_around": 4,
            "mark_time": 5, "squat": 6, "turn_roll": 7, "turn_pitch": 8,
            "turn_yaw": 9, "3_axis": 10, "pee": 11, "sit": 12,
            "wave_hand": 13, "stretch": 14, "wave_body": 15, "swing": 16,
            "pray": 17, "seek": 18, "handshake": 19, "push_up": 20
        }
        
        # MOZGÁSI SEBESSÉG (Ezt állíthatod, max kb. 20-25)
        self.speed = 15 

        if HARDWARE_AVAILABLE:
            self.dog = DOGZILLA()
            self.get_logger().info('✅ Dogzilla Node kész: Trükkök + Séta aktív!')
        else:
            self.dog = DOGZILLA()
            self.get_logger().warn('⚠️ Dogzilla Node kész: SZIMULÁLT MÓD')

    def get_battery_level(self):
        """
        Biztonságos akku olvasás. Ha hiba van (SerialException),
        nem omlik össze, hanem 0-t vagy szimulált adatot ad vissza.
        """
        # Ha nincs hardver könyvtár, szimulálunk
        if not HARDWARE_AVAILABLE:
             return round(random.uniform(7.4, 8.4), 2)

        # Ha van hardver, megpróbáljuk olvasni
        try:
            if hasattr(self.dog, 'read_battery'):
                battery = self.dog.read_battery()
                
                # Néha a hardver None-t vagy furcsa értéket adhat vissza
                if battery is None:
                    return 0.0
                
                return battery
            
        except Exception as e:
            # Itt kapjuk el a SerialException-t!
            self.get_logger().warn(f"Akku olvasási hiba (Serial): {e}")
            return 0.0
            
        return 0.0

    def timer_callback(self):
        # 1. Adatok összegyűjtése egy struktúrába
        telemetry_data = {
            "status": "online",
            "battery_voltage": self.get_battery_level(),
            "current_speed": self.speed,
            "mode": "manual" 
        }

        # 2. Átalakítás JSON Stringgé
        json_payload = json.dumps(telemetry_data)

        # 3. Publikálás a ROS hálózatba
        msg = String()
        msg.data = json_payload
        self.state_publisher.publish(msg)

    # --- JAVÍTVA: Behúzás korrigálva, most már az osztály része ---
    def listener_callback(self, msg):
        cmd = msg.data.lower().strip().replace(" ", "_")
        self.get_logger().info(f'Parancs: "{cmd}"')

        # Ha nincs hardver kapcsolat, ne omoljon össze
        if not hasattr(self, 'dog'):
            self.get_logger().warn("Nincs hardver kapcsolat!")
            return

        try:
            # --- 1. MOZGÁS PARANCSOK (Javított hívásokkal) ---
            if cmd == "forward":
                if hasattr(self.dog, 'forward'): 
                    self.dog.forward(self.speed)
                else:
                    self.get_logger().error("HIBA: Nincs 'forward' függvény!")

            elif cmd == "backward":
                if hasattr(self.dog, 'back'): 
                    self.dog.back(self.speed)

            elif cmd == "left":
                if hasattr(self.dog, 'left'): 
                    self.dog.left(self.speed)

            elif cmd == "right":
                if hasattr(self.dog, 'right'): 
                    self.dog.right(self.speed)

            elif cmd == "turn_left":
                if hasattr(self.dog, 'turnleft'): 
                    self.dog.turnleft(self.speed)

            elif cmd == "turn_right":
                if hasattr(self.dog, 'turnright'): 
                    self.dog.turnright(self.speed)

            elif cmd == "stop":
                if hasattr(self.dog, 'stop'): 
                    self.dog.stop()
                else:
                    try: self.dog.move(0, 0)
                    except: pass

            # --- 2. TRÜKKÖK ---
            elif cmd in self.action_map:
                action_code = self.action_map[cmd]
                self.dog.action(action_code)

            else:
                self.get_logger().warn(f"Ismeretlen parancs: {cmd}")

        except Exception as e:
            self.get_logger().error(f"Kritikus hiba a végrehajtásban: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = DogzillaAllInOneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop hívás kilépéskor
        if hasattr(node, 'dog'):
            if hasattr(node.dog, 'stop'): node.dog.stop()
            elif hasattr(node.dog, 'move'): 
                try: node.dog.move(0,0) # Vagy (0,0,0) attól függ mi működik
                except: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
