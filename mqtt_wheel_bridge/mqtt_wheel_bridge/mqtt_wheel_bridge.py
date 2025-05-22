import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Bool, String
from geometry_msgs.msg import Twist, Point
# from mqtt_wheel_msgs.msg import WheelData

from paho.mqtt import client as mqtt
import time
from numpy import interp

MQTT_HOST = "192.168.0.3"
MQTT_PORT = 1883

# ROS to MQTT translation for wheel messages
class WheelBridge(Node):

    wheels = ["frontleft", "frontright", "backleft", "backright"] # MQTT names of the wheels
    logger = None # rclpy logger
    client : mqtt.Client # MQTT client
    
    
    frontleft_data =  Point()
    frontright_data = Point()
    backleft_data =   Point()
    backright_data =  Point()
    drive_mode = "ackermann"

    def __init__(self):
        super().__init__("mqtt_wheel_bridge")
        self.logger = self.get_logger()
        
        self.frontleft_pub = self.create_publisher( Point, f"frontleft/data", 10)
        self.frontright_pub = self.create_publisher(Point, f"frontright/data", 10)
        self.backleft_pub = self.create_publisher(  Point, f"backleft/data", 10)
        self.backright_pub = self.create_publisher( Point, f"backright/data", 10)

        # set up power, steer, and brake subscribers for each wheel
        for wheel in self.wheels:
            self.create_subscription(Point, f"{wheel}/control", eval(f"self.{wheel}_cb"), 10)
            
        self.create_subscription(Empty, "calibrate_steering", self.calibrate_cb, 10)
        self.create_subscription(String, "drive_mode", self.drive_mode_cb, 10)
        self.create_subscription(Bool, "direction", self.direction_cb, 10)

        # MQTT client connect
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="wheel_bridge")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        while not self.client.is_connected():
            try:
                
                self.client.connect(MQTT_HOST, MQTT_PORT)
                self.client.loop_start()
            except TimeoutError:
                self.logger.log("Waiting for MQTT...", 20)
        

    def ms_to_throttle(self, ms : float):
        negative = ms < 0
        if ms == 0:
            return 0
        
        if negative:
            ms = ms * -1
        
        ms_values = [0, 0.447,2.68]
        throttle_values = [0, 97,115]
        out = interp(ms, ms_values, throttle_values)
        
        if negative:
            out = out * -1
        return out
    
    def ms_to_turns_s(self, ms : float):
        negative = ms < 0
        if ms == 0:
            return 0
        
        if negative:
            ms = ms * -1
        
        ms_values = [0, 0.447,2.68]
        turns_values = [0, 0.28,1.67]
        out = interp(ms, ms_values, turns_values) * 2
        
        if negative:
            out = out * -1
        return out
    
    def wheel_cb(self, wheel : str, control : Point, convert_pwm : bool = False):
        if convert_pwm:
            self.client.publish(f"/{wheel}/power", self.ms_to_throttle(control.x))
        else:
            self.client.publish(f"/{wheel}/power", self.ms_to_turns_s(control.x))
        self.client.publish(f"/{wheel}/steer", control.z)
        
    def frontleft_cb(self, msg :  Point): self.wheel_cb("frontleft", msg)
    def frontright_cb(self, msg : Point): self.wheel_cb("frontright", msg)
    def backleft_cb(self, msg :   Point): self.wheel_cb("backleft", msg, convert_pwm=True)
    def backright_cb(self, msg :  Point): self.wheel_cb("backright", msg, convert_pwm=True)
    
    def calibrate_cb(self, msg):
        self.client.publish(f"/calibrate", "")
        
    def drive_mode_cb(self, msg):
        self.drive_mode = msg.data
    
    def direction_cb(self, msg):
        if self.drive_mode == "rotate":
            if msg.data:
                self.client.publish("/frontleft/direction", "1")
                self.client.publish("/frontright/direction", "0")
                self.client.publish("/backleft/direction", "1")
                self.client.publish("/backright/direction", "0")
            else:
                self.client.publish("/frontleft/direction", "0")
                self.client.publish("/frontright/direction", "1")
                self.client.publish("/backleft/direction", "0")
                self.client.publish("/backright/direction", "1")
        else:
            if msg.data:
                self.client.publish("/frontleft/direction", "1")
                self.client.publish("/frontright/direction", "1")
                self.client.publish("/backleft/direction", "1")
                self.client.publish("/backright/direction", "1")
            else:
                self.client.publish("/frontleft/direction", "0")
                self.client.publish("/frontright/direction", "0")
                self.client.publish("/backleft/direction", "0")
                self.client.publish("/backright/direction", "0")
        
    
    
        
        
    def on_connect(self, client: mqtt.Client, userdata, flags, reason_code, properties):
        for wheel in self.wheels:
            self.client.subscribe(f"/{wheel}/encoder")
            self.client.subscribe(f"/{wheel}/velocity")
            self.client.subscribe(f"/{wheel}/debug")
            
    def on_message(self, client: mqtt.Client, userdata, message: mqtt.MQTTMessage):
        wheel = None
        for w in self.wheels:
            if w in message.topic:
                wheel = w 
                break
            
        if wheel is not None:
            if message.topic.endswith("encoder"):
                exec(f"self.{wheel}_data.x = float(message.payload)")
            elif message.topic.endswith("velocity"):
                exec(f"self.{wheel}_data.z = float(message.payload)")
            # elif message.topic.endswith("debug"):
            #     exec(f"self.{wheel}_data.debug = message.payload")
            exec(f"self.{wheel}_pub.publish(self.{wheel}_data)")
            
    
            
    
def main(args=None):
    rclpy.init(args=args)
    node = WheelBridge()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()