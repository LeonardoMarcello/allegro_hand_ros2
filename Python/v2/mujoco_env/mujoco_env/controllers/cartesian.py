import numpy as np

class CartesianImpedanceController:
    def __init__(
        self,
        fingers,
        x_des
    ) -> None:

        self._fingers = fingers
        self._x_des = x_des
        self._kp =[ np.array([.5,.5,.5,.5]),
                    np.array([.5,.5,.5,.5]),
                    np.array([.5,.5,.5,.5]),
                    np.array([.5,.5,.5,.5])]
        self._kv = [np.array([.01,.01,.01,.01]),
                    np.array([.01,.01,.01,.01]),
                    np.array([.01,.01,.01,.01]),
                    np.array([.01,.01,.01,.01])]
        self._cartesian_stiffness =[np.array([60,60,60]),
                                    np.array([60,60,60]),
                                    np.array([60,60,60]),
                                    np.array([60,60,60])]
        self._cartesian_damping = [ np.array([5,5,5]),
                                    np.array([5,5,5]),
                                    np.array([5,5,5]),
                                    np.array([5,5,5])]
    
    def set_current_state(self, current_state):
        for i in range(len(self._fingers)):
            self._fingers[i].set_q(current_state['q'][4*i:4*(i+1)])
            self._fingers[i].set_dq(current_state['dq'][4*i:4*(i+1)])

    def set_target(self, x_des):
        self._x_des = x_des

 
    def compute(self, dt) -> np.ndarray:
        """
        Run the robot controller.

        Parameters:
            target (numpy.ndarray): The desired target joint positions or states for the robot.
                                   The size of `target` should be (n_joints,) where n_joints is the number of robot joints.
            ctrl (numpy.ndarray): Control signals for the robot actuators from `mujoco._structs.MjData.ctrl` of size (nu,).
        """

        tau = np.zeros((16,))
        for i in range(len(self._fingers)):
            # get end effector (ith finger) pose
            x_ee_i = self._fingers[i].get_T_w_ee()[0:3,3]
            # compute attitude error
            error_ee_i = self._x_des[3*i:3*(i+1)] - x_ee_i

            # Inverse Kinematics using the combined vector
            J = self._fingers[i].get_J_ee()[:3,:]
            JTpinv = np.linalg.pinv(J.T)
            N = (np.eye(4) - J.T @ JTpinv)
            
            # apply torque
            tau_task = J.T @ (  np.diag(self._cartesian_stiffness[i]) @ (error_ee_i.reshape(-1,1)) -
                                np.diag(self._cartesian_damping[i]) @ J @ self._fingers[i].get_dq().reshape(-1,1)
                             )
            tau_null = N @ (  - np.diag(self._kp[i]) @ self._fingers[i].get_q().reshape(-1,1)+
                                np.diag(self._kv[i]) @ self._fingers[i].get_dq().reshape(-1,1)
                            )

            tau_gravity = self._fingers[i].get_G().reshape(-1,1)

            tau[4*i:4*(i+1)] = (tau_task + tau_null + tau_gravity).flatten()
        return tau
