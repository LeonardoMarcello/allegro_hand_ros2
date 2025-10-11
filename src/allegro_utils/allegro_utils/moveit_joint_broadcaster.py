import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter

class JointRelay(Node):
    def __init__(self):
        super().__init__('joint_broadcaster')

        # Declare parameters for input/output topics with default values
        self.declare_parameter('input_topic', '/joint_states')
        self.declare_parameter('output_topic', '/allegroHand_0/joint_cmd')

        # Get the parameter values
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.desired_order = [
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
        # Create subscriber and publisher
        self.sub = self.create_subscription(
            JointState,
            self.input_topic,
            self.callback,
            1
        )
        self.pub = self.create_publisher(
            JointState,
            self.output_topic,
            1
        )

        # Storage for last received message
        self.last_msg = None

        # Timer for publishing
        rate = 50 # (Hz)
        self.timer = self.create_timer(1/rate, self.timer_callback)

        self.get_logger().info(f"Relaying from {self.input_topic} → {self.output_topic} at {rate} Hz")

    def callback(self, msg):
        # Store the latest message
        self.last_msg = msg

    def timer_callback(self):
        if self.last_msg is not None:
            # reorder topic

            # Build a map from name → index
            idx_map = {name: i for i, name in enumerate(self.last_msg.name)}

            # New arrays
            new_msg = JointState()
            new_msg.header = self.last_msg.header
            new_msg.name = self.desired_order
            new_msg.position = [self.last_msg.position[idx_map[n]] for n in self.desired_order]
            new_msg.velocity = [self.last_msg.velocity[idx_map[n]] for n in self.desired_order]
            new_msg.effort   = [self.last_msg.effort[idx_map[n]]   for n in self.desired_order]
            self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = JointRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
