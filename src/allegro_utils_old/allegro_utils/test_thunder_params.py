import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
import numpy as np

import sys
sys.path.append("./include/ahand_finger_generatedFiles/build") # Where the .so file is located. 
from thunder_ahand_finger_py import thunder_ahand_finger

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


class JointRelay(Node):
    def __init__(self):
        super().__init__('wave')

        # Declare parameters for input/output topics with default values
        self.declare_parameter('input_topic', '/bag/allegroHand_0/joint_states_cmd')
        self.declare_parameter('output_topic', '/allegroHand_0/torque_cmd')

        # Create subscriber and publisher
        self.sub = self.create_subscription(
            JointState,
            self.input_topic,
            self.callback,
            1
        )
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(
            JointState,
            self.output_topic,
            1
        )

        self.thunder_finger = thunder_ahand_finger()


    def callback(self, msg):
        cmd = JointState()
        
        # get joint
        q_des = np.array(msg.position)
        qd_des = np.array(msg.velocity)
        qdd_des = np.array(msg.acceleration)

        # get regressor
        self.thunder_finger.set_q(q_des)
        self.thunder_finger.set_dq(qd_des)
        self.thunder_finger.set_dqr(qd_des)
        self.thunder_finger.set_ddqr(qdd_des)
        M = self.thunder_ahand.get_M() 
        C = self.thunder_ahand.get_C()
        G = self.thunder_ahand.get_G()
        D = np.diag([1,1,1,1])
        F = np.diag([1,1,1,1])

        kp = np.array([1,1,1,1])
        kv = np.array([1,1,1,1])

        tau = M @ ()
        Y_d = np.diag(qd_des)
        Y_s = np.diag(2/(1+np.exp(-20*qd_des) - 1))

        Y = np.hstack([Y, Y_d, Y_s])

        # Comuted Torque

        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = [0.0]*16
        self.msg.velocity = [0.0]*16
        self.msg.effort   = [0.0]*16
        for i in range(len(self.msg.position)):
            self.msg.position[i] = q_target[i]
            self.msg.velocity[i] = dq_target[i]
            self.msg.effort[i] = 0.0

        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = JointRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
