import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String

from paho.mqtt import client as mqtt

MQTT_HOST = "192.168.0.3"
MQTT_PORT = 1883

# ROS to MQTT translation for wheel messages
class WheelBridge(Node):

    wheels = ["frontleft", "frontright", "backleft", "backright"] # MQTT names of the wheels
    logger = None # rclpy logger
    client : mqtt.Client # MQTT client

    def __init__(self):
        super().__init__("mqtt_wheel_bridge")
        self.logger = self.get_logger()

        # set up power, steer, and brake subscribers for each wheel
        for wheel in self.wheels:
            self.create_subscription(Int32, f"{wheel}/power", eval(f"self.{wheel}_power_cb"), 10)
            self.create_subscription(Int32, f"{wheel}/steer", eval(f"self.{wheel}_steer_cb"), 10)
            self.create_subscription(Int32, f"{wheel}/brake", eval(f"self.{wheel}_brake_cb"), 10)

        # MQTT client connect
        self.client = mqtt.Client(client_id="wheel_bridge")
        self.client.connect(MQTT_HOST, MQTT_PORT)
        self.client.loop_start()

    # general power callback
    def power_cb(self, msg : Int32, wheel : str):
        self.client.publish(f"/{wheel}/power", msg.data)

    # general steering callback
    def steer_cb(self, msg : Int32, wheel : str):
        self.client.publish(f"/{wheel}/steer", msg.data)

    # general brake callback
    def brake_cb(self, msg : Int32, wheel : str):
        self.client.publish(f"/{wheel}/brake", msg.data)

    # per wheel subscribers (this is ugly but I don't know a way around it)
    def frontleft_power_cb(self, msg : Int32):
        self.power_cb(msg, "frontleft")
    def frontright_power_cb(self, msg : Int32):
        self.power_cb(msg, "frontright")
    def backleft_power_cb(self, msg : Int32):
        self.power_cb(msg, "backleft")
    def backright_power_cb(self, msg : Int32):
        self.power_cb(msg, "backright")

    def frontleft_steer_cb(self, msg : Int32):
        self.steer_cb(msg, "frontleft")
    def frontright_steer_cb(self, msg : Int32):
        self.steer_cb(msg, "frontright")
    def backleft_steer_cb(self, msg : Int32):
        self.steer_cb(msg, "backleft")
    def backright_steer_cb(self, msg : Int32):
        self.steer_cb(msg, "backright")
    
    def frontleft_brake_cb(self, msg : Int32):
        self.brake_cb(msg, "frontleft")
    def frontright_brake_cb(self, msg : Int32):
        self.brake_cb(msg, "frontright")
    def backleft_brake_cb(self, msg : Int32):
        self.brake_cb(msg, "backleft")
    def backright_brake_cb(self, msg : Int32):
        self.brake_cb(msg, "backright")






def main(args=None):
    rclpy.init(args=args)
    node = WheelBridge()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()