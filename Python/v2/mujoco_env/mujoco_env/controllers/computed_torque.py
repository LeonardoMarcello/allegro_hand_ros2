import numpy as np

class ComputedTorqueController:
    def __init__(
        self,
        fingers,
        q_des
    ) -> None:

        self._fingers = fingers

        self._q_des = q_des
        self._dq_des = np.zeros_like(q_des)
        self._ddq_des = np.zeros_like(q_des)

        self._kp =[ np.array([8,8,8,8]),
                    np.array([8,8,8,8]),
                    np.array([8,8,8,8]),
                    np.array([8,8,8,8])]
        self._kv = [np.array([.5,.5,.5,.5]),
                    np.array([.5,.5,.5,.5]),
                    np.array([.5,.5,.5,.5]),
                    np.array([.5,.5,.5,.5])]

    def set_current_state(self, current_state):
        for i in range(len(self._fingers)):
            self._fingers[i].set_q(current_state['q'][4*i:4*(i+1)])
            self._fingers[i].set_dq(current_state['dq'][4*i:4*(i+1)])

    def set_target(self, q_des, dq_des = None, ddq_des = None):
        self._q_des = q_des
        if dq_des is not None:
            self._dq_des = dq_des
        if ddq_des is not None:
            self._ddq_des = ddq_des



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
            # get joint config (ith finger)
            q_i = self._fingers[i].get_q()
            # compute joint error
            error_q_i = self._q_des[3*i:3*(i+1)] - q_i

            # apply torque
            tau_task = ( np.diag(self._kp[i]) @ (error_q_i.reshape(-1,1)) -
                         np.diag(self._kv[i]) @ self._fingers[i].get_dq().reshape(-1,1)
                       )
            tau_mass = self._fingers[i].get_M()
            tau_coriolis = self._fingers[i].get_C() @ self._dq_des
            tau_gravity = self._fingers[i].get_G().reshape(-1,1)

            tau[4*i:4*(i+1)] = (tau_task + tau_mass + tau_coriolis + tau_gravity).flatten()
        return tau
