import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
import numpy as np

class JointRelay(Node):
    def __init__(self):
        super().__init__('wave')

        # Declare parameters for input/output topics with default values
        self.declare_parameter('output_topic', '/allegroHand_0/joint_cmd')
        self.declare_parameter('speed', 0.5)  # <-- speed as parameter (rad/s)
        self.speed = self.speed = self.get_parameter('speed').get_parameter_value().double_value

        # Get the parameter values
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(
            JointState,
            self.output_topic,
            1
        )

        self.msg = JointState()
        #self.msg.header = self.last_msg.header
        self.msg.name = [
            'joint_0_0',
            'joint_1_0',
            'joint_2_0',
            'joint_3_0',
            'joint_4_0',
            'joint_5_0',
            'joint_6_0',
            'joint_7_0',
            'joint_8_0',
            'joint_9_0',
            'joint_10_0',
            'joint_11_0',
            'joint_12_0',
            'joint_13_0',
            'joint_14_0',
            'joint_15_0'
        ]
        self.open_hand = np.deg2rad([0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    90.0,
                                    0.0,
                                    0.0]).tolist()
        # Timer for publishing
        rate = 50 # (Hz)


        self.timer = self.create_timer(1/rate, self.timer_callback)
        self.t0 = self.get_clock().now()

        self.get_logger().info(f"Waving publisher at {rate} Hz and {self.speed} rad/s")


    def timer_callback(self):
        t = self.get_clock().now()
        # New arrays
        self.msg.position = self.open_hand
        
        self.msg.position[1] += np.pi/180*(25 + 25*np.sin(self.speed*(t-self.t0).nanoseconds*1e-9))
        self.msg.position[5] += np.pi/180*(25 + 25*np.sin(self.speed*(t-self.t0).nanoseconds*1e-9 + np.pi/4))
        self.msg.position[9] += np.pi/180*(25 + 25*np.sin(self.speed*(t-self.t0).nanoseconds*1e-9 + np.pi/2))

        self.msg.velocity = [0.0]*len(self.msg.position)
        self.msg.effort   = [0.0]*len(self.msg.position)
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = JointRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
