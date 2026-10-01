#!/usr/bin/env python3
"""
jacobian_wrench_compare.py

Estimates the end-effector wrench from joint torques via tau = J(q)^T F,
and compares it against a measured wrench (e.g. wrist F/T sensor) - offline
analysis of a recorded rosbag.

No KDL. The Jacobian is assembled directly from tf plus a one-time read of
the robot URDF for the joint axes and types.


ros2 run robot_state_publisher robot_state_publisher --ros-args     -p robot_description:="$(cat /home/leo/Desktop/allegro_hand_ros2_ws/src/allegro_hand_description/urdf/allegro_hand_description_right_B.urdf)"     -p use_sim_time:=true

ros2 bag play vertical_press --clock --topics /joint_states /sensor_1/netft_data

python3 wrench_jacobian_compare.py --ros-args -p use_sim_time:=true


"""
import xml.etree.ElementTree as ET
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import message_filters
import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String


def quat_to_rot(q) -> np.ndarray:
    """Converts a geometry_msgs Quaternion to a 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 1e-12 else 0.0
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


def parse_active_joints(urdf_xml: str, base_frame: str, sensor_frame: str):
    """Walk the URDF tree from sensor_frame back to base_frame."""
    root = ET.fromstring(urdf_xml)
    joints = {}
    for jel in root.findall('joint'):
        name = jel.get('name')
        axis_el = jel.find('axis')
        # The axis is defined in the joint frame (which is the child link frame)
        axis = np.array([float(v) for v in axis_el.get('xyz', '1 0 0').split()]
                         if axis_el is not None else [1.0, 0.0, 0.0])
        joints[name] = dict(
            type=jel.get('type'),
            parent=jel.find('parent').get('link'),
            child=jel.find('child').get('link'),
            axis=axis,
        )

    child_to_joint = {jdef['child']: (name, jdef) for name, jdef in joints.items()}
    chain, link = [], sensor_frame
    while link != base_frame:
        if link not in child_to_joint:
            raise RuntimeError(
                f'No URDF path from "{base_frame}" to "{sensor_frame}" '
                f'(stuck at "{link}") - check the frame names.')
        name, jdef = child_to_joint[link]
        chain.append((name, jdef))
        link = jdef['parent']
    chain.reverse()
    return [(n, jdef) for n, jdef in chain if jdef['type'] != 'fixed']


class JacobianWrenchCompare(Node):

    def __init__(self):
        super().__init__('jacobian_wrench_compare')

        self.declare_parameter('base_frame', 'link_0_0')
        self.declare_parameter('sensor_frame', 'link_3_0')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('wrench_topic', '/sensor_1/netft_data')
        self.declare_parameter('sync_slop', 0.02)
        self.declare_parameter('tf_timeout', 0.1)
        self.declare_parameter('urdf_path', '')

        self.base_frame = self.get_parameter('base_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout').value)
        
        if not self.sensor_frame:
            self.get_logger().error('Set -p sensor_frame:=<tf frame of your F/T sensor>.')

        self.active_joints = []

        # URDF Loading Logic
        urdf_path = self.get_parameter('urdf_path').value
        if urdf_path:
            self.get_logger().info(f'Loading URDF directly from {urdf_path}')
            try:
                with open(urdf_path, 'r') as f:
                    self._process_urdf(f.read())
            except Exception as e:
                self.get_logger().error(f'Failed to read URDF file: {e}')
        else:
            self.get_logger().info('Waiting for /robot_description topic...')
            urdf_qos = QoSProfile(depth=1)
            urdf_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            urdf_qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.create_subscription(String, '/robot_description', self._on_urdf, urdf_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Synchronize incoming joint states and F/T measurements
        js_sub = message_filters.Subscriber(
            self, JointState, self.get_parameter('joint_states_topic').value)
        wr_sub = message_filters.Subscriber(
            self, WrenchStamped, self.get_parameter('wrench_topic').value)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [js_sub, wr_sub], queue_size=100,
            slop=self.get_parameter('sync_slop').value)
        self.sync.registerCallback(self._on_synced)

        self.pub_est = self.create_publisher(WrenchStamped, '~/wrench_estimated', 10)
        self.pub_err = self.create_publisher(WrenchStamped, '~/wrench_error', 10)
        self.pub_tau_est = self.create_publisher(JointState, '~/tau_estimated', 10)
        self.pub_tau_err = self.create_publisher(JointState, '~/tau_err', 10)
    def _on_urdf(self, msg: String):
        self._process_urdf(msg.data)

    def _process_urdf(self, urdf_xml: str):
        if self.active_joints or not self.sensor_frame:
            return
        try:
            active = parse_active_joints(urdf_xml, self.base_frame, self.sensor_frame)
        except RuntimeError as ex:
            self.get_logger().error(str(ex))
            return
        self.active_joints = active
        self.get_logger().info(
            f'Initialized Jacobian chain. {self.base_frame} -> {self.sensor_frame}: '
            f'{[n for n, _ in self.active_joints]}')

    def _jacobian(self):
        """6xn Jacobian at sensor_frame, expressed ENTIRELY in sensor_frame axes."""
        lookup_time = Time() 
        n = len(self.active_joints)
        J = np.zeros((6, n))
        
        for col, (name, jdef) in enumerate(self.active_joints):
            try:
                # TARGET is sensor_frame, SOURCE is the joint (child link)
                # This gives us the joint's pose expressed IN the sensor's coordinate frame
                pt = self.tf_buffer.lookup_transform(
                    self.sensor_frame, jdef['child'], lookup_time, timeout=self.tf_timeout)
            except TransformException as ex:
                return None
                
            # pi = position of the joint IN the sensor frame
            pi = np.array([pt.transform.translation.x, pt.transform.translation.y, pt.transform.translation.z])
            
            # z = axis of rotation IN the sensor frame
            z = quat_to_rot(pt.transform.rotation) @ jdef['axis']

            if jdef['type'] in ('revolute', 'continuous'):
                # Because we are in the sensor frame, p_tip is exactly [0, 0, 0].
                # Therefore, the vector from the joint to the tip is (0 - pi) = -pi
                J[0:3, col] = np.cross(z, -pi)
                J[3:6, col] = z
            elif jdef['type'] == 'prismatic':
                J[0:3, col] = z
                
        return J

    def _on_synced(self, js: JointState, wrench: WrenchStamped):
        if not self.active_joints or not js.effort:
            return

        idx_by_name = {n: i for i, n in enumerate(js.name)}
        try:
            tau = np.array([js.effort[idx_by_name[n]] for n, _ in self.active_joints])
        except KeyError:
            return

        J = self._jacobian()
        if J is None:
            return
            
        # Because J was built entirely in the sensor frame, the resulting F_est 
        # is automatically expressed in the sensor frame (link_3_0).
        # v = J dq
        # tau = J.T w
        F_est = np.linalg.pinv(J.T) @ tau 
        w = np.array([wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z,
                     1e-3*wrench.wrench.torque.x, 1e-3*wrench.wrench.torque.y, 1e-3*wrench.wrench.torque.z] )
        tau_est = - J.T @ w

        # Publish estimated wrench
        out = WrenchStamped()
        out.header.stamp = js.header.stamp
        out.header.frame_id = self.sensor_frame
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = F_est[:3]
        out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = F_est[3:]
        self.pub_est.publish(out)

        # Publish wrench error (Estimated - Measured)
        err = WrenchStamped()
        err.header = out.header
        err.wrench.force.x = F_est[0] - wrench.wrench.force.x
        err.wrench.force.y = F_est[1] - wrench.wrench.force.y
        err.wrench.force.z = F_est[2] - wrench.wrench.force.z
        err.wrench.torque.x = F_est[3] - wrench.wrench.torque.x
        err.wrench.torque.y = F_est[4] - wrench.wrench.torque.y
        err.wrench.torque.z = F_est[5] - wrench.wrench.torque.z
        self.pub_err.publish(err)


        # Publish the reconstructed torques for plotting
        js_est = JointState()
        js_est.header.stamp = js.header.stamp
        js_est.name = [n for n, _ in self.active_joints]
        js_est.effort = tau_est.tolist()
        self.pub_tau_est.publish(js_est)

        # Publish the reconstructed torques for plotting
        js_err = JointState()
        js_err.header.stamp = js.header.stamp
        js_err.name = [n for n, _ in self.active_joints]
        js_err.effort = (tau - tau_est).tolist()
        self.pub_tau_err.publish(js_err)

def main():
    rclpy.init()
    
    # Optional: Suppress TF_OLD_DATA warnings from C++ if they are still spamming you
    logger = rclpy.logging.get_logger('tf2_ros')
    logger.set_level(rclpy.logging.LoggingSeverity.ERROR)
    
    node = JacobianWrenchCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()