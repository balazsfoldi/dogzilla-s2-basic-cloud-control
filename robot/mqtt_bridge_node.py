import json
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import paho.mqtt.client as mqtt


class RosMqttBridgeNode(Node):
    """
    ROS2 <-> MQTT bridge:

    MQTT topicok:
      - robot/cmd/vel : JSON parancsok (vx, vy, wz)
      - robot/cmd     : sima string parancs (pl. "sit")
      - robot/state   : állapot (String)

    ROS2 topicok:
      - /cmd_vel      : Twist
      - /robot/cmd    : String
      - /robot_state  : String
    """

    def __init__(self):
        super().__init__("ros_mqtt_bridge")

        # Paraméterek
        self.declare_parameter('broker.host', '155.98.37.82')
        self.declare_parameter('broker.port', 31883)

        mqtt_host = self.get_parameter('broker.host').get_parameter_value().string_value
        mqtt_port = self.get_parameter('broker.port').get_parameter_value().integer_value

        self.get_logger().info(f"Kapcsolódás MQTT brokerhez: {mqtt_host}:{mqtt_port}")

        # ROS publ és sub
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_mode_pub = self.create_publisher(String, "robot/cmd", 10)
        self.state_sub = self.create_subscription(String, "robot_state", self.state_callback, 10)

        # MQTT kliens
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        try:
            self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Nem sikerült csatlakozni az MQTT brokerhez: {e}")
            raise

        self.get_logger().info("RosMqttBridgeNode elindult.")

    # ROS → MQTT
    def state_callback(self, msg: String):
        payload = msg.data
        self.mqtt_client.publish("robot/state", payload)
        self.get_logger().info(f"[ROS→MQTT] robot/state → {payload}")

    # MQTT → ROS
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT kapcsolódás sikeres.")
            client.subscribe("robot/cmd/vel")
            client.subscribe("robot/cmd")
        else:
            self.get_logger().error(f"MQTT kapcsolódási hiba: rc={rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode("utf-8")

            if msg.topic == "robot/cmd" and "ping" in payload_str:
                # Ha "ping" jön, azonnal visszaküldjük "pong"-ként az időbélyeggel együtt
                # Így mérhető, mennyi időt töltött a rendszerben
                response = payload_str.replace("ping", "pong")
                self.mqtt_client.publish("robot/state", response)
                self.get_logger().info(f"[PING] Válasz küldve: {response}")
                return # Ne menjen tovább a ROS felé, ez csak mérés

            if msg.topic == "robot/cmd/vel":
                data = json.loads(payload_str)
                vx = float(data.get("vx", 0.0))
                vy = float(data.get("vy", 0.0))
                wz = float(data.get("wz", 0.0))

                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                twist.angular.z = wz

                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"[MQTT→ROS] /cmd_vel → vx={vx}, vy={vy}, wz={wz}")

            elif msg.topic == "robot/cmd":
                cmd_msg = String()
                cmd_msg.data = payload_str
                self.cmd_mode_pub.publish(cmd_msg)
                self.get_logger().info(f"[MQTT→ROS] /robot/cmd → '{payload_str}'")

        except Exception as e:
            self.get_logger().error(f"[MQTT→ROS] Hiba: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RosMqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
