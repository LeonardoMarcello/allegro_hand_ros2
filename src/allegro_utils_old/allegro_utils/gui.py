import sys
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.task import Future
import numpy as np

from std_msgs.msg import Float64MultiArray, Bool, String
from sensor_msgs.msg import JointState
from allegro_utils.AHAND import *

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QGridLayout, QCheckBox, QPushButton
)
from PyQt5.QtCore import Qt, QTimer


# ------------------------------------------------------------------------------------
# Paths and Constants
# ------------------------------------------------------------------------------------
current_file = Path(__file__).resolve()
pkg_share = current_file.parent

URDF_PATH = os.path.join(pkg_share, 'description', 'urdf', 'allegro_hand_description_right_B.urdf')
XML_PATH = os.path.join(pkg_share, 'description', 'urdf', 'allegro_hand_description_right_B.xml')
MESH_DIR = os.path.join(pkg_share, 'description', 'urdf', 'meshes', '')

FINGERS = ['index', 'middle', 'ring', 'thumb']
JOINTS = [
    'joint_0_0','joint_1_0','joint_2_0','joint_3_0',      # index
    'joint_4_0','joint_5_0','joint_6_0','joint_7_0',      # middle
    'joint_8_0','joint_9_0','joint_10_0','joint_11_0',    # ring
    'joint_12_0','joint_13_0','joint_14_0','joint_15_0'   # thumb
]
A = 0.05  # Nm


# ------------------------------------------------------------------------------------
# ROS 2 Node
# ------------------------------------------------------------------------------------
class AllegroForcePublisher(Node):
    def __init__(self):
        super().__init__('allegro_force_gui_node')

        # Declare parameters
        self.declare_parameter('input_topic', '/allegroHand_0/joint_states')
        self.declare_parameter('output_topic', '/allegroHand_leo/torque_cmd')
        self.declare_parameter('control_topic', '/allegroHand_leo/lib_cmd')

        # Get topic names
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value

        # Create publishers/subscribers
        self.force_pub = self.create_publisher(JointState, self.output_topic, 10)
        self.ctr_pub = self.create_publisher(String, self.control_topic, 10)
        self.state_sub = self.create_subscription(
            JointState,
            self.input_topic,
            self.state_callback,
            10
        )

        # Initialize
        self.force_values = [0.0] * len(JOINTS)
        self.comp_enabled = False
        self.ahand = AHAND(URDF_PATH, MESH_DIR)

        self.get_logger().info(f"Subscribed to: {self.input_topic}")
        self.get_logger().info(f"Publishing torques to: {self.output_topic}")

    def publish_forces(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = np.zeros((len(JOINTS),)).tolist()
        msg.velocity = np.zeros((len(JOINTS),)).tolist()

        if self.comp_enabled:
            tau_ctrl = np.array(self.force_values) + self.ahand.getGravityVector()
            msg.effort = tau_ctrl.tolist()
        else:
            msg.effort = self.force_values

        self.force_pub.publish(msg)

    def state_callback(self, msg):
        """Callback triggered when new JointState messages arrive."""
        self.get_logger().debug("JointState message received")
        try:
            self.ahand.q = np.array(msg.position)
            self.ahand.dq = np.array(msg.velocity)
            self.ahand.updateConfig()
        except Exception as e:
            self.get_logger().error(f"Error in state_callback: {e}")


# ------------------------------------------------------------------------------------
# PyQt5 GUI
# ------------------------------------------------------------------------------------
class ForceControlGUI(QWidget):
    def __init__(self, node: AllegroForcePublisher):
        super().__init__()
        self.node = node
        self.setWindowTitle("Allegro Hand Force Command GUI")

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.layout.addWidget(QLabel("<h2>Allegro Hand Joint Force Control</h2>"))

        # Grid of sliders for 4 fingers × 4 joints
        grid = QGridLayout()
        self.sliders = []

        # Finger headers
        for c, finger in enumerate(FINGERS):
            label = QLabel(f"<b>{finger}</b>")
            label.setAlignment(Qt.AlignCenter)
            grid.addWidget(label, 0, 2*c + 1)

        # Joint sliders
        for joint in range(4):
            for finger in range(4):
                idx = 4*finger + joint
                label = QLabel(f"{JOINTS[idx]}")
                slider = QSlider(Qt.Horizontal)
                precision = 20
                slider.setRange(-int(precision), int(precision))
                slider.setValue(0)
                slider.setTickPosition(QSlider.TicksBelow)
                slider.setTickInterval(precision)
                slider.valueChanged.connect(lambda val, i=idx: self.update_force(i, A * val / precision))

                # Labels for slider range
                tick_layout = QHBoxLayout()
                tick_layout.addWidget(QLabel(f"-{A}"))
                tick_layout.addStretch()
                tick_layout.addWidget(QLabel("0"))
                tick_layout.addStretch()
                tick_layout.addWidget(QLabel(f"{A}"))

                grid.addWidget(label, 1 + joint, 2 * finger)
                grid.addWidget(slider, 1 + joint, 2 * finger + 1)
                self.sliders.append(slider)
        self.layout.addLayout(grid)

        # Checkboxes
        self.comp_checkbox = QCheckBox("Enable Gravity Compensation")
        self.comp_checkbox.stateChanged.connect(self.toggle_compensation)
        self.layout.addWidget(self.comp_checkbox)

        self.ctrl_checkbox = QCheckBox("Control Activated")
        self.ctrl_checkbox.stateChanged.connect(self.toggle_control)
        self.layout.addWidget(self.ctrl_checkbox)

        # Buttons
        reset_btn = QPushButton("Reset Forces")
        reset_btn.clicked.connect(self.reset_forces)
        self.layout.addWidget(reset_btn)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close_gui)
        self.layout.addWidget(close_btn)

        # Timer: publish + spin ROS
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)  # 10 Hz

    def spin_ros(self):
        """Process ROS events and publish current forces."""
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node.publish_forces()

    def update_force(self, idx, value):
        self.node.force_values[idx] = float(value)

    def toggle_compensation(self, state):
        self.node.comp_enabled = state == Qt.Checked

    def toggle_control(self, state):
        msg = String()
        msg.data = "on" if state == Qt.Checked else "off"
        self.node.ctr_pub.publish(msg)

    def reset_forces(self):
        for slider in self.sliders:
            slider.setValue(0)
        self.node.force_values = [0.0] * 16

    def close_gui(self):
        """Stop timer, deactivate control, close GUI, and shutdown ROS."""
        msg = String()
        msg.data = "off"
        self.node.ctr_pub.publish(msg)
        self.timer.stop()
        self.node.get_logger().info("Closing Allegro Force GUI...")
        self.close()


# ------------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AllegroForcePublisher()

    app = QApplication(sys.argv)
    gui = ForceControlGUI(node)
    gui.show()

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
