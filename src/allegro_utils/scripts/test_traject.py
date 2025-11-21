#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
import numpy as np
import copy

JOINT_LIMITS = np.array([
    [-0.3, 0.3],    # Index 0
    [-0.01, 1.6],   # Index 1
    [-0.07, 1.86],  # Index 2
    [-0.02, 2.01],  # Index 3
    [-0.26, 0.26],  # Middle 0
    [-0.21, 1.79],  # Middle 1 (real)
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
# limit config
# 1: [0.26, 1.790, 1.368, 0]
# 2: [0.26, 1.628, 1.432, 0.269]
# 3: [0.26, 0.644, 1.528, 1.449]
# 4: [0.26, 0.331, 1.507, 1.850]
# TRAJECT_JOINT_LIMITS[i][j][k] -> ith traject, j-th joint, k = 0:1 lower:upper bound
TRAJECT_JOINT_LIMITS = [[[-0.26,0.26], [0,1.790], [0,1.368], [0,0.000]],
                        [[-0.26,0.26], [0,1.628], [0,1.432], [0,0.269]],
                        [[-0.26,0.26], [0,0.644], [0,1.528], [0,1.449]],
                        [[-0.26,0.26], [0,0.331], [0,1.507], [0,1.850]]]

TRAJECT_TIME_PERIOD_S = 20  #uration of a single selected traject

# Test Traject search grid
#FREQUENCY = np.array([0, 0.2, 0.4, 0.6, 0.8])#, 1, 1.2])
#PHASE = np.array([0, np.pi/3, np.pi/4, np.pi/2, 3*np.pi/4, np.pi])

# Test Traject Slow  search grid
FREQUENCY = np.array([0, 0.01, 0.05])
PHASE = np.array([0, np.pi/3, np.pi/4, np.pi/2, 3*np.pi/4, np.pi])


# Test Traject Very Slow  search grid
#FREQUENCY = np.array([0.01])#, 1, 1.2])
#PHASE = np.array([0, np.pi/2, np.pi/3, np.pi/4])

# utils
def alpha(t,t0, a = 1):
    # a: if t-t0 > 6a then alpha is almost 0
    return 1/(1+np.exp((t-t0)/a))

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
        self.q_bar = np.array((JOINT_LIMITS[:,1] + JOINT_LIMITS[:,0])/2)   # Mid position used to target configuration
        # Timer for publishing
        rate = 50 # (Hz)
        self.timer = self.create_timer(1/rate, self.timer_callback)
        self.t0 = self.get_clock().now()

        self.traject = {"q_bar" : np.zeros_like(self.q_bar),
                        "A": np.zeros_like(self.q_bar),
                        "f": np.zeros_like(self.q_bar),
                        "phi": np.zeros_like(self.q_bar),
                        "idx": 0}
        # init traject
        self.traject["idx"] = np.random.choice([0,1,2,3])
        self.traject["q_bar"][[4,5,6,7]] = np.array([(TRAJECT_JOINT_LIMITS[self.traject["idx"]][i][1] + TRAJECT_JOINT_LIMITS[self.traject["idx"]][i][0])/2 for i in [0,1,2,3]])
        self.traject["A"][[4,5,6,7]] = np.array([0.9 * (TRAJECT_JOINT_LIMITS[self.traject["idx"]][i][1] - TRAJECT_JOINT_LIMITS[self.traject["idx"]][i][0])/2 for i in [0,1,2,3]])
        self.traject["f"][[4,5,6,7]] = np.array([np.random.choice(FREQUENCY) for _ in [0,1,2,3]])
        self.traject["phi"][[4,5,6,7]] = np.array([np.random.choice(PHASE) for _ in [0,1,2,3]])
        

        # Variable to switch across traject
        self.traject_2 = copy.deepcopy(self.traject)
        self.enable_switch_traject = True
        self.switch_t0 = self.get_clock().now()

    def init_pos(self, msg):
        # Store the latest message
        if self.oninit:
            for i in range(len(msg.position)):
                self.traject["q_bar"][i] = msg.position[i]
            print("initialized")
            self.oninit=False

    def timer_callback(self):
        t = self.get_clock().now()
        delta_t = (t-self.t0).nanoseconds*1e-9
        q_target = np.zeros_like(self.q_bar)
        dq_target = np.zeros_like(self.q_bar)
        q_target_2 = np.zeros_like(self.q_bar)
        dq_target_2 = np.zeros_like(self.q_bar)

        # =============== Define here target trajectory =========================================
        # get traject
        for i in [4,5,6,7]:
            q_bar = self.traject["q_bar"][i]
            amplitude = self.traject["A"][i]
            frequency = self.traject["f"][i]
            phase = self.traject["phi"][i]
            omega = 2 * np.pi * frequency

            q_target[i] = amplitude * np.sin(omega * t.nanoseconds*1e-9 + phase) + q_bar
            dq_target[i] = omega * amplitude * np.cos(omega * t.nanoseconds*1e-9 + phase)

        # if transition
        if delta_t > TRAJECT_TIME_PERIOD_S - 6 and delta_t < TRAJECT_TIME_PERIOD_S + 6:
            # if init transition
            if (self.enable_switch_traject):
                self.traject_2["idx"] = np.random.choice([0,1,2,3])
                self.traject_2["q_bar"][[4,5,6,7]] = np.array([(TRAJECT_JOINT_LIMITS[self.traject_2["idx"]][i][1] + TRAJECT_JOINT_LIMITS[self.traject_2["idx"]][i][0])/2 for i in [0,1,2,3]])
                self.traject_2["A"][[4,5,6,7]]= np.array([0.9 * (TRAJECT_JOINT_LIMITS[self.traject_2["idx"]][i][1] - TRAJECT_JOINT_LIMITS[self.traject_2["idx"]][i][0])/2 for i in [0,1,2,3]])
                self.traject_2["f"][[4,5,6,7]]= np.array([np.random.choice(FREQUENCY) for _ in [0,1,2,3]])
                self.traject_2["phi"][[4,5,6,7]] = np.array([np.random.choice(PHASE) for _ in [0,1,2,3]])
                self.enable_switch_traject = False
                self.switch_t0 = t.nanoseconds*1e-9 + 6


            for i in [4,5,6,7]:
                q_bar = self.traject_2["q_bar"][i]
                amplitude = self.traject_2["A"][i]
                frequency = self.traject_2["f"][i]
                phase = self.traject_2["phi"][i]
                omega = 2 * np.pi * frequency

                q_target_2[i] = amplitude * np.sin(omega * t.nanoseconds*1e-9 + phase) + q_bar
                dq_target_2[i] = omega * amplitude * np.cos(omega * t.nanoseconds*1e-9 + phase)

            # apply transition
            q_target = q_target*alpha(t.nanoseconds*1e-9, self.switch_t0) + q_target_2*(1-alpha(t.nanoseconds*1e-9, self.switch_t0))

        # if transition complete
        elif delta_t > TRAJECT_TIME_PERIOD_S + 6:
            for i in [4,5,6,7]:
                q_bar = self.traject_2["q_bar"][i]
                amplitude = self.traject_2["A"][i]
                frequency = self.traject_2["f"][i]
                phase = self.traject_2["phi"][i]
                omega = 2 * np.pi * frequency

                q_target[i] = amplitude * np.sin(omega * t.nanoseconds*1e-9 + phase) + q_bar
                dq_target[i] = omega * amplitude * np.cos(omega * t.nanoseconds*1e-9 + phase)
            self.traject = copy.deepcopy(self.traject_2)
            self.enable_switch_traject = True
            self.t0 = t
        # =======================================================================================

        # Publish Trajectory
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

if __name__ == '__main__':
    main()
