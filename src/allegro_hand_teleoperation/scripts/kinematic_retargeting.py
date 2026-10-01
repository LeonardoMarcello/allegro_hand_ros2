#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from mediapipe_msgs.msg import HandLandmarkArray
from visualization_msgs.msg import Marker, MarkerArray

# NEW: Trajectory controller message imports
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

import os
import sys
import glob
import yaml

# Load the fingers model (Thunder Ahand)
# ---------------------------------------------------------------------------
rel_paths = glob.glob("/home/leo/Desktop/inhand manipulation/mujoco_sim/thunder/*/build")
for rel_path in rel_paths:
    if os.path.exists(rel_path):
        sys.path.append(os.path.abspath(rel_path))
    else:
        print(f"Warning: path '{rel_path}' does not exist.")

from thunder_ahand_finger_py import thunder_ahand_finger
from thunder_ahand_thumb_py import thunder_ahand_thumb


# 3) Define utilities
def load_params(robot, config_path):
    with open(config_path, 'r') as f:
        config_robot = yaml.safe_load(f)

    # Dynamic parameters
    robot.set_par_DYN(config_robot['par_DYN'])
    robot.set_par_Dl(config_robot['par_Dl'])
    robot.set_par_REG(config_robot['par_REG'])
    robot.set_par_REG_red(config_robot['par_REG_red'])
    if hasattr(robot, "set_par_Ia"):
        robot.set_par_Ia(config_robot['par_Ia'])
    # Other parameters
    if hasattr(robot, "set_par_Ln2EE"):
        robot.set_par_Ln2EE(config_robot['par_Ln2EE'])
    if hasattr(robot, "set_par_world2L0"):
        robot.set_par_world2L0(config_robot['par_world2L0'])
    if hasattr(robot, "set_par_gravity"):
        robot.set_par_gravity(config_robot['par_gravity'])


class AllegroRetargetingNode(Node):
    def __init__(self):
        super().__init__('allegro_retargeting_node')

        # 1. ROS 2 Setup & Parameters
        self.declare_parameter('input_topic', 'hand_landmarks')
        self.declare_parameter('output_topic', '/allegroHand_0/joint_cmd')
        self.declare_parameter('trajectory_topic', '/allegro_position_controller/joint_trajectory')
        self.declare_parameter('palm_name', 'palm_link')

        self.landmark_topic_name = self.get_parameter('input_topic').get_parameter_value().string_value
        self.joint_topic_name = self.get_parameter('output_topic').get_parameter_value().string_value
        self.trajectory_topic_name = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.palm_name = self.get_parameter('palm_name').get_parameter_value().string_value

        # NEW: Define joint names matching ros2_control config (joint_0_0 ... joint_15_0)
        self.joint_names = [f'joint_{i}_0' for i in range(16)]

        # Sub/Pub
        self.sub = self.create_subscription(HandLandmarkArray, self.landmark_topic_name, self.callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'retargeting_debug_markers', 10)
        self.pub = self.create_publisher(JointState, self.joint_topic_name, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, self.trajectory_topic_name, 10)

        self.mirror_view = True

        # 2. Retargeting Parameters
        self.beta = 1.6         
        self.eps = 0.02         
        self.nu1 = 1e-4         
        self.nu2 = 3e-2         
        self.gamma = 2.5e-3

        self.alpha = 0.2  # Low-pass filter constant

        # State tracking
        self.q_current = np.zeros(16)
        self.q_filtered = np.zeros(16)

        # Define Allegro joint bounds (16 joints)
        self.bounds = [(-0.3, 0.3), (-0.01, 1.6), (-0.07, 1.86),
                       (-0.26, 0.26), (-0.21, 1.79), (-0.12, 1.86),
                       (-0.26, 0.29), (-0.21, 1.79), (-0.12, 1.86),
                       (0.00, 1.78), (-0.26, 1.65), (-0.05, 1.85), (-0.09, 1.80)]

        # 3. Setup Kinematics
        ring = thunder_ahand_finger()
        index = thunder_ahand_finger()
        middle = thunder_ahand_finger()
        thumb = thunder_ahand_thumb()
        config_path = "/home/leo/Desktop/allegro_hand_ros2_ws/src/allegro_hand_ros2_controllers/config/thunder/"
        load_params(index, config_path + "ahand_index_identification_par.yaml")
        load_params(middle, config_path + "ahand_middle_identification_par.yaml")
        load_params(ring,  config_path + "ahand_ring_identification_par.yaml")
        load_params(thumb, config_path + "ahand_thumb_identification_par.yaml")
        self.fingers = [index, middle, ring, thumb]
        print("Allegro Retargeting Node Initialized")

    def callback(self, msg):
        # 1. Extract raw landmarks
        h_landmarks_raw = np.array([[p.x, p.y, p.z] for p in msg.landmarks[0].world_landmarks])
        if len(h_landmarks_raw) < 21: return

        # 2. Alignment via Rotation (-90Z, 90Y)
        r_align = R.from_euler('zy', [-90, 90], degrees=True)
        h_landmarks_rot = r_align.apply(h_landmarks_raw - h_landmarks_raw[0])

        # 3. Apply Mirror Flag if requested (flip X-axis)
        if self.mirror_view:
            h_landmarks_rot[:, 0] *= -1

        # 4. Shift origin to Middle Finger Metacarpal (Landmark 9)
        mcp_middle = h_landmarks_rot[9]
        h_landmarks = h_landmarks_rot - mcp_middle

        # 5. Extract positions from the centered landmarks
        h_pos = {
            'thumb_cmc': h_landmarks[1],
            'thumb': h_landmarks[4],
            'index': h_landmarks[8],
            'middle': h_landmarks[12],
            'ring': h_landmarks[16]
        }

        # 6. Extract Human Vectors (ri)
        h_vectors = [
            (h_pos['thumb'] - h_pos['index']),
            (h_pos['thumb'] - h_pos['middle']),
            (h_pos['thumb'] - h_pos['ring']),
            (h_pos['index'] - h_pos['middle']),
            (h_pos['index'] - h_pos['ring']),
            (h_pos['ring'] - h_pos['middle']),
        ]

        # 7. Solve Optimization (SLSQP)
        start = self.get_clock().now()
        res = minimize(
            self.cost_function,
            self._collapse_costraints(self.q_filtered),
            args=(h_vectors,),
            method='SLSQP',
            bounds=self.bounds,
            options={'ftol': 1e-3, 'maxiter': 15}
        )
        end = self.get_clock().now()
        #self.get_logger().info(f"Solve time: {(end - start).nanoseconds / 1e6:.2f} ms")

        if res.success:
            q_target = self._expand_costraints(res.x)

            # 8. Low-pass Filter
            self.q_filtered = self.alpha * q_target + (1 - self.alpha) * self.q_filtered

            # 9. Publish Result
            self.publish_joints(self.q_filtered)
            self.publish_debug_markers(self.q_filtered, h_landmarks)

    def cost_function(self, q_a_reduced, h_vectors):
        cost = 0.0
        q_a = self._expand_costraints(q_a_reduced)
        a_pos = self._get_allegro_pos(q_a)
        a_vectors = [
            (a_pos['thumb'] - a_pos['index']),
            (a_pos['thumb'] - a_pos['middle']),
            (a_pos['thumb'] - a_pos['ring']),
            (a_pos['index'] - a_pos['middle']),
            (a_pos['index'] - a_pos['ring']),
            (a_pos['ring']  - a_pos['middle']),
        ]

        for i, (ri_h, ri_a) in enumerate(zip(h_vectors, a_vectors)):
            di = np.linalg.norm(ri_h)
            hat_ri_h = ri_h / (di + 1e-6)

            if di > self.eps:
                s_di = 1.0
                f_di = self.beta * di
            else:
                s_di = 200.0 if i < 3 else 400.0           
                f_di = self.nu1 if i < 3 else self.nu2     

            target_vec = f_di * hat_ri_h
            cost += 0.5 * s_di * np.sum((ri_a - target_vec)**2)

        cost += self.gamma * np.sum(q_a**2) 
        return cost

    def publish_joints(self, q):
        now = self.get_clock().now().to_msg()

        # 1. Publish JointState (for visualization or state broad-casting)
        msg = JointState()
        msg.header.stamp = now
        msg.name = self.joint_names
        msg.position = q.tolist()
        self.pub.publish(msg)

        # 2. Publish JointTrajectory (for ros2_control JointTrajectoryController)
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = now
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = q.tolist()

        # Target duration for step execution (50 ms matches ~20 Hz update loop)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50_000_000

        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)

    def publish_debug_markers(self, a_q, h_landmarks_full):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i in range(len(self.fingers)):
            self.fingers[i].set_q(a_q[4*i : 4*(i+1)])
            a_pos = self.fingers[i].get_T_w_ee()[0:3, 3]
            a_marker = self.create_marker(i, "allegro", a_pos, [0.0, 0.0, 1.0, 0.7], 0.012, now)
            marker_array.markers.append(a_marker)

        for i, pt in enumerate(h_landmarks_full):
            if i in [4, 8, 12, 16]:
                color = [1.0, 0.5, 0.0, 0.9]
                scale = 0.02
            else:
                color = [0.0, 1.0, 0.0, 0.5]
                scale = 0.008
            dot = self.create_marker(i, "human_hand_dots", pt, color=color, scale=scale, timestamp=now)
            marker_array.markers.append(dot)

        self.marker_pub.publish(marker_array)

    def create_marker(self, id, ns, pos, color, scale, timestamp):
        m = Marker()
        m.header.frame_id = self.palm_name
        m.header.stamp = timestamp
        m.ns = ns
        m.id = id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m
    
    def _expand_costraints(self, q_red):
        q_full = np.zeros(16)
        q_full[0:3] = q_red[0:3]   
        q_full[4:7] = q_red[3:6]   
        q_full[8:11] = q_red[6:9]  
        q_full[12:16] = q_red[9:13] 
        q_full[3], q_full[7], q_full[11] = q_full[2], q_full[6], q_full[10]
        return q_full

    def _collapse_costraints(self, q_full):
        q_red = np.zeros(13)
        q_red[0:3] = q_full[0:3]   
        q_red[3:6] = q_full[4:7]   
        q_red[6:9] = q_full[8:11]  
        q_red[9:13] = q_full[12:16] 
        return q_red

    def _get_allegro_pos(self, q):
        a_pos = {'thumb_cmc': self.fingers[3].get_T_w_1()}
        name = ['index', 'middle', 'ring', 'thumb']
        for i in range(len(self.fingers)):
            q_finger = np.array(q[4*i : 4*(i+1)])
            self.fingers[i].set_q(q_finger)
            T_w_ee = self.fingers[i].get_T_w_ee()
            pos = T_w_ee[0:3, 3]
            a_pos[name[i]] = pos
        return a_pos


def main():
    rclpy.init()
    node = AllegroRetargetingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()