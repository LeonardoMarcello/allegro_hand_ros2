#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
import numpy as np

JOINT_LIMITS = np.array([
    [-0.3, 0.3],    # Index 0
    [-0.01, 1.6],   # Index 1
    [-0.07, 1.86],  # Index 2
    [-0.02, 2.01],  # Index 3
    [-0.26, 0.26],  # Middle 0
    [-0.21, 1.79],  # Middle 1
    [-0.12, 1.86],  # Middle 2
    [-0.21, 1.85],  # Middle 3
    [-0.26, 0.29],  # Ring 0
    [-0.21, 1.79],  # Ring 1
    [-0.12, 1.86],  # Ring 2
    [-0.21, 1.85],  # Ring 3
    [0.00, 1.78],   # Thumb 0
    [-0.26, 1.65],  # Thumb 1
    [-0.05, 1.85],  # Thumb 2
    [-0.09, 1.80],  # Thumb 3
])

FREQUENCY = np.array([0, 0.2, 0.4, 0.6, 0.8, 1, 1.2])
PHASE = np.array([0, np.pi/3, np.pi/4, np.pi/2, 3*np.pi/4, np.pi])

class JointRelay(Node):
    def __init__(self):
        super().__init__('wave')

        # Declare parameters for input/output topics with default values
        self.declare_parameter('output_topic', '/allegroHand_0/joint_cmd')
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(
            JointState,
            self.output_topic,
            1
        )
        # Create subscriber and publisher
        self.sub = self.create_subscription(
            JointState,
            '/allegroHand_0/joint_states',
            self.init_pos,
            1
        )
        self.oninit=True

        self.msg = JointState()
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

        # Useful Predifined positions
        self.open_hand = np.zeros(16)
        self.open_hand[13] = np.pi/2

        self.q_bar = (JOINT_LIMITS[:,1] + JOINT_LIMITS[:,0])/2   # Mid position used to target configuration
        self.q_target = np.zeros_like(self.q_bar)

        # Timer for publishing
        rate = 50 # (Hz)
        self.timer = self.create_timer(1/rate, self.timer_callback)
        self.t0 = self.get_clock().now()

        self.frequency = np.zeros_like(self.q_bar)
        self.phase = np.zeros_like(self.q_bar)
        #self.get_logger().info(f"Waving publisher at {rate} Hz and {self.speed} rad/s")

    def init_pos(self, msg):
        # Store the latest message
        if self.oninit:
            for i in range(len(msg.position)):
                self.q_target[i] = msg.position[i]
            print("initialized")
            self.oninit=False



    def timer_callback(self):
        t = self.get_clock().now()
        delta_t = (t-self.t0).nanoseconds*1e-9
        dq_target = np.zeros_like(self.q_bar)

        # =============== Define here target trajectory =========================================
        if delta_t > 15:
            for i in [4,5,6,7]:
                self.frequency[i] = np.random.choice(FREQUENCY)
                self.phase[i] = np.random.choice(PHASE)
            self.t0 = t

        for i in [4,5,6,7]:
            amplitude = 0.3 * (self.q_bar[i] - JOINT_LIMITS[i,0])
            frequency = self.frequency[i]
            phase = self.phase[i]
            omega = 2 * np.pi * frequency
            self.q_target[i] = amplitude * np.sin(omega * delta_t + phase) + self.q_bar[i]
            dq_target[i] = omega * amplitude * np.cos(omega * delta_t)

        # =======================================================================================

        # Publish Trajectory
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = [0.0]*16
        self.msg.velocity = [0.0]*16
        self.msg.effort   = [0.0]*16
        for i in range(len(self.msg.position)):
            self.msg.position[i] = self.q_target[i]
            self.msg.velocity[i] = dq_target[i]
            self.msg.effort[i] = 0.0

        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = JointRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
