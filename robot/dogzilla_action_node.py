import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

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

class DogzillaAllInOneNode(Node):
    def __init__(self):
        super().__init__('dogzilla_action_node')
        
        self.subscription = self.create_subscription(
            String,
            'robot/cmd',
            self.listener_callback,
            10)

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

    def listener_callback(self, msg):
        cmd = msg.data.lower().strip().replace(" ", "_")
        self.get_logger().info(f'Parancs: "{cmd}"')

        # 1. ESET: MOZGÁS PARANCSOK
        if cmd == "forward":
            self.dog.move(self.speed, 0, 0) # X=sebesség, Y=0, R=0
        elif cmd == "backward":
            self.dog.move(-self.speed, 0, 0) # X=negatív
        elif cmd == "left":
            self.dog.move(0, self.speed, 0) # Y=sebesség (oldalazás balra)
        elif cmd == "right":
            self.dog.move(0, -self.speed, 0) # Y=negatív (oldalazás jobbra)
        elif cmd == "turn_left":
            self.dog.move(0, 0, self.speed) # R=forgás
        elif cmd == "turn_right":
            self.dog.move(0, 0, -self.speed)
        elif cmd == "stop":
            self.dog.move(0, 0, 0) # Minden 0 -> Megáll

        # 2. ESET: TRÜKKÖK
        elif cmd in self.action_map:
            action_code = self.action_map[cmd]
            self.dog.action(action_code)

        else:
            self.get_logger().warn(f"Ismeretlen parancs: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = DogzillaAllInOneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.dog.move(0,0,0) # Biztonsági megállás
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
