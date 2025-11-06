#!/usr/bin/env python3
import sys
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
import numpy as np
from functools import partial

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from allegro_utils.AHAND import *

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QGridLayout, QCheckBox, QPushButton, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer


# ------------------------------------------------------------------------------------
# Paths and Constants
# ------------------------------------------------------------------------------------
current_file = Path(__file__).resolve()
pkg_share = current_file.parent
URDF_PATH = os.path.join(pkg_share, 'description', 'urdf', 'allegro_hand_description_right_B.urdf')
MESH_DIR = os.path.join(pkg_share, 'description', 'urdf', 'meshes', '')

FINGERS = ['index', 'middle', 'ring', 'thumb']
JOINTS = [
    'joint_0_0','joint_1_0','joint_2_0','joint_3_0',
    'joint_4_0','joint_5_0','joint_6_0','joint_7_0',
    'joint_8_0','joint_9_0','joint_10_0','joint_11_0',
    'joint_12_0','joint_13_0','joint_14_0','joint_15_0'
]
A = 0.05  # Nm


# ------------------------------------------------------------------------------------
# ROS 2 Node
# ------------------------------------------------------------------------------------
class AllegroForcePublisher(Node):
    def __init__(self):
        super().__init__('allegro_force_gui_node')

        self.declare_parameter('input_topic', '/allegroHand_0/joint_states')
        self.declare_parameter('output_topic', '/allegroHand_0/torque_cmd')
        self.declare_parameter('control_topic', '/allegroHand_0/lib_cmd')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value

        self.force_pub = self.create_publisher(JointState, self.output_topic, 10)
        self.ctr_pub = self.create_publisher(String, self.control_topic, 10)
        self.state_sub = self.create_subscription(JointState, self.input_topic, self.state_callback, 10)

        self.force_values = [0.0] * len(JOINTS)
        self.comp_enabled = False
        self.ahand = AHAND(URDF_PATH, MESH_DIR)

        self.get_logger().info(f"Publishing torques to: {self.output_topic}")

    def publish_forces(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [0.0] * len(JOINTS)
        msg.velocity = [0.0] * len(JOINTS)

        if self.comp_enabled:
            tau_ctrl = np.array(self.force_values) + self.ahand.getGravityVector()
            msg.effort = tau_ctrl.tolist()
        else:
            msg.effort = self.force_values

        self.force_pub.publish(msg)

    def state_callback(self, msg):
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
        self.resize(1600, 800)  # width x height in pixels


        self.layout = QVBoxLayout(self)
        self.layout.addWidget(QLabel("<h2>Allegro Hand Joint Force Control</h2>"))

        grid = QGridLayout()
        self.sliders = []
        self.force_labels = []

        precision = 100

        # Header row
        for c, finger in enumerate(FINGERS):
            header = QLabel(f"<b>{finger}</b>")
            header.setAlignment(Qt.AlignCenter)
            grid.addWidget(header, 0, c)

        # Populate sliders per finger
        for joint in range(4):
            for finger in range(4):
                idx = 4 * finger + joint
                vbox = QVBoxLayout()

                # joint label
                joint_label = QLabel(JOINTS[idx])
                joint_label.setAlignment(Qt.AlignCenter)

                # slider + buttons
                slider = QSlider(Qt.Horizontal)
                slider.setRange(-precision, precision)
                slider.setValue(0)
                slider.setTickPosition(QSlider.TicksBelow)
                slider.setTickInterval(precision)
                slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                slider.valueChanged.connect(lambda val, i=idx: self.update_force(i, A * val / precision))

                up_btn = QPushButton("▲")
                down_btn = QPushButton("▼")
                up_btn.setFixedWidth(25)
                down_btn.setFixedWidth(25)
                up_btn.clicked.connect(partial(self.adjust_slider, slider, +1))
                down_btn.clicked.connect(partial(self.adjust_slider, slider, -1))

                force_label = QLabel("+0.000 Nm")
                force_label.setAlignment(Qt.AlignCenter)
                force_label.setMinimumWidth(160)
                self.force_labels.append(force_label)


                btn_layout = QHBoxLayout()
                btn_layout.addWidget(down_btn)
                btn_layout.addWidget(force_label)
                btn_layout.addWidget(up_btn)


                vbox.addWidget(joint_label)
                vbox.addWidget(slider)
                vbox.addLayout(btn_layout)
                grid.addLayout(vbox, joint + 1, finger)

                self.sliders.append(slider)

        
        self.layout.addLayout(grid)

        # Checkboxes and buttons
        self.comp_checkbox = QCheckBox("Enable Gravity Compensation")
        self.comp_checkbox.stateChanged.connect(self.toggle_compensation)
        self.layout.addWidget(self.comp_checkbox)

        self.ctrl_checkbox = QCheckBox("Control Activated")
        self.ctrl_checkbox.stateChanged.connect(self.toggle_control)
        self.layout.addWidget(self.ctrl_checkbox)

        reset_btn = QPushButton("Reset Forces")
        reset_btn.clicked.connect(self.reset_forces)
        self.layout.addWidget(reset_btn)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close_gui)
        self.layout.addWidget(close_btn)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)

    def adjust_slider(self, slider, step):
        slider.setValue(max(slider.minimum(), min(slider.maximum(), slider.value() + step)))

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node.publish_forces()
        for i, val in enumerate(self.node.force_values):
            reordered_idx = {0:0,   1:4,    2:8,        3:12,
                             4:1,   5:5,    6:9,        7:13,
                             8:2,   9:6,    10:10,      11:14,
                             12:3,  13:7,   14:11,      15:15}
            self.force_labels[reordered_idx[i]].setText(f"{val*1000:+.1f} Nmm")

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
        self.node.force_values = [0.0] * len(JOINTS)

    def resizeEvent(self, event):
        for slider in self.sliders:
            slider.setFixedWidth(max(100, self.width() // 8))
        super().resizeEvent(event)

    def close_gui(self):
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
