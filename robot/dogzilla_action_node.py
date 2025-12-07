import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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
    # Mock osztály teszteléshez (hogy PC-n se omoljon össze)
    class DOGZILLA:
        def action(self, code): print(f"[SIM] Action: {code}")
        def forward(self, speed): print(f"[SIM] Forward: {speed}")
        def back(self, speed): print(f"[SIM] Back: {speed}")
        def left(self, speed): print(f"[SIM] Left: {speed}")
        def right(self, speed): print(f"[SIM] Right: {speed}")
        def turnleft(self, speed): print(f"[SIM] TurnL: {speed}")
        def turnright(self, speed): print(f"[SIM] TurnR: {speed}")
        def stop(self): print("[SIM] STOP")
        def read_battery(self): return 8.2

class DogzillaAllInOneNode(Node):
    def __init__(self):
        super().__init__('dogzilla_action_node')
        self.get_logger().info('--- DOGZILLA FULL NODE (CMD + VEL) INDUL ---')
        
        # 1. String parancsok figyelése (Trükkök, egyszerű mozgás)
        self.sub_str = self.create_subscription(
            String,
            'robot/cmd',
            self.listener_callback,
            10)

        # 2. Twist (cmd_vel) figyelése (Joystick / Folyamatos mozgás)
        self.sub_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # 3. Telemetria publikáló
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        
        # 4. Időzítő a telemetriához (2 másodpercenként)
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Trükkök leképezése
        self.action_map = {
            "lie_down": 1, "stand_up": 2, "crawl": 3, "turn_around": 4,
            "mark_time": 5, "squat": 6, "turn_roll": 7, "turn_pitch": 8,
            "turn_yaw": 9, "3_axis": 10, "pee": 11, "sit": 12,
            "wave_hand": 13, "stretch": 14, "wave_body": 15, "swing": 16,
            "pray": 17, "seek": 18, "handshake": 19, "push_up": 20
        }
        
        # Alapértelmezett sebesség
        self.speed = 10

        # Hardver inicializálás
        if HARDWARE_AVAILABLE:
            self.dog = DOGZILLA()
            self.get_logger().info('Hardver csatlakoztatva')
        else:
            self.dog = DOGZILLA()
            self.get_logger().warn('️Szimulált mód')

    def get_battery_level(self):
        """ Biztonságos akku olvasás """
        if not HARDWARE_AVAILABLE: return 8.2
        try:
            if hasattr(self.dog, 'read_battery'):
                val = self.dog.read_battery()
                return val if val is not None else 0.0
        except Exception:
            return 0.0
        return 0.0

    def timer_callback(self):
        """ Telemetria küldése JSON formátumban """
        telemetry_data = {
            "status": "online",
            "battery_voltage": self.get_battery_level(),
            "current_speed": self.speed,
            "mode": "hybrid (cmd+vel)" 
        }
        msg = String()
        msg.data = json.dumps(telemetry_data)
        self.state_publisher.publish(msg)

    def cmd_vel_callback(self, msg):
        """ 
        ROS Twist üzenet feldolgozása (Joystick logika).
        Ez teszi lehetővé a robot/cmd/vel MQTT téma használatát.
        """
        if not hasattr(self, 'dog'): return

        vx = msg.linear.x
        wz = msg.angular.z
        
        # Küszöbérték (hogy a zaj miatt ne mozogjon)
        threshold = 0.1

        try:
            if vx > threshold:
                if hasattr(self.dog, 'forward'): self.dog.forward(self.speed)
            elif vx < -threshold:
                if hasattr(self.dog, 'back'): self.dog.back(self.speed)
            elif wz > threshold:
                # ROS koordináta rendszerben a pozitív Z forgás balra van
                if hasattr(self.dog, 'turnleft'): self.dog.turnleft(self.speed)
            elif wz < -threshold:
                # Negatív Z forgás jobbra van
                if hasattr(self.dog, 'turnright'): self.dog.turnright(self.speed)
            else:
                # Ha minden 0, akkor megállás
                if hasattr(self.dog, 'stop'): self.dog.stop()
                
        except Exception as e:
            self.get_logger().error(f"CmdVel Hiba: {e}")

    def listener_callback(self, msg):
        """ String parancsok feldolgozása (Trükkök és diszkrét mozgás)
        """
        cmd = msg.data.lower().strip().replace(" ", "_")
        self.get_logger().info(f'Parancs: "{cmd}"')

        if not hasattr(self, 'dog'): return

        try:
            # MOZGÁS (Biztonságos függvényhívásokkal)
            if cmd == "forward":
                if hasattr(self.dog, 'forward'): self.dog.forward(self.speed)
            elif cmd == "backward":
                if hasattr(self.dog, 'back'): self.dog.back(self.speed)
            elif cmd == "left":
                if hasattr(self.dog, 'left'): self.dog.left(self.speed)
            elif cmd == "right":
                if hasattr(self.dog, 'right'): self.dog.right(self.speed)
            elif cmd == "turn_left":
                if hasattr(self.dog, 'turnleft'): self.dog.turnleft(self.speed)
            elif cmd == "turn_right":
                if hasattr(self.dog, 'turnright'): self.dog.turnright(self.speed)
            elif cmd == "stop":
                if hasattr(self.dog, 'stop'): self.dog.stop()
            
            # TRÜKKÖK
            elif cmd in self.action_map:
                self.dog.action(self.action_map[cmd])
            
            else:
                self.get_logger().warn(f"Ismeretlen: {cmd}")

        except Exception as e:
            self.get_logger().error(f"Hiba: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DogzillaAllInOneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Biztonsági megállás kilépéskor
        if hasattr(node, 'dog') and hasattr(node.dog, 'stop'):
            node.dog.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
