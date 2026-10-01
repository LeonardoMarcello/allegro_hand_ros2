from dm_control import mjcf

import numpy as np

class Arm():
    def __init__(self, xml_path, eef_site_name, attachment_site_name, joint_names = None, name: str = None):
        self._mjcf_root = mjcf.from_path(xml_path)
        if name:
            self._mjcf_root.model = name

        # Find MJCF elements that will be exposed as attributes.
        if joint_names is None:
            self._joints = self.mjcf_model.find_all('joint')
        else:
            self._joints = [self._mjcf_root.find('joint', name) for name in joint_names]
        
        self._eef_site = self._mjcf_root.find('site', eef_site_name)
        self._attachment_site = self._mjcf_root.find('site', attachment_site_name)

    @property
    def joints(self):
        """List of joint elements belonging to the arm."""
        return self._joints

    @property
    def eef_site(self):
        """Wrist site of the arm (attachment point for the hand)."""
        return self._eef_site

    @property
    def mjcf_model(self):
        """Returns the `mjcf.RootElement` object corresponding to this robot."""
        return self._mjcf_root
    
    def attach_tool(self, child, pos: list = [0, 0, 0], quat: list = [1, 0, 0, 0]) -> mjcf.Element:
        frame = self._attachment_site.attach(child)
        frame.pos = pos
        frame.quat = quat
        return frame
    
    def get_eef_pose(self, physics):
        ee_pos = physics.bind(self._eef_site).xpos
        ee_quat = self._mat2quat(physics.bind(self._eef_site).xmat.reshape(3, 3))
        ee_pose = np.concatenate((ee_pos, ee_quat))
        return ee_pose



    def _mat2quat(self,rmat):
        """
        Converts given rotation matrix to quaternion.

        Args:
            rmat (np.array): 3x3 rotation matrix

        Returns:
            np.array: (x,y,z,w) float quaternion angles
        """
        M = np.asarray(rmat).astype(np.float32)[:3, :3]

        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array(
            [
                [m00 - m11 - m22, np.float32(0.0), np.float32(0.0), np.float32(0.0)],
                [m01 + m10, m11 - m00 - m22, np.float32(0.0), np.float32(0.0)],
                [m02 + m20, m12 + m21, m22 - m00 - m11, np.float32(0.0)],
                [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22],
            ]
        )
        K /= 3.0
        # quaternion is Eigen vector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        inds = np.array([3, 0, 1, 2])
        q1 = V[inds, np.argmax(w)]
        if q1[0] < 0.0:
            np.negative(q1, q1)
        inds = np.array([1, 2, 3, 0])
        return q1[inds]