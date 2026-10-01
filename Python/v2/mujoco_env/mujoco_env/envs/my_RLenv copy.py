import time
import os
import numpy as np
from dm_control import mjcf
import mujoco.viewer
from mujoco_env.arenas import StandardArena
from mujoco_env.robots import Arm, Hand
from Python.v2.mujoco_env.mujoco_env.utils.interface import JointEffortController

import gymnasium as gym
from gymnasium import spaces


class MyRLEnv(gym.Env):

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": None,
    }  # TODO add functionality to render_fps

    def __init__(self, render_mode=None):
        # TODO come up with an observation space that makes sense
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64
        )

        # TODO come up with an action space that makes sense
        self.action_space = spaces.Box(
            low=-0.1, high=0.1, shape=(6,), dtype=np.float64
        )

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self._render_mode = render_mode

        ############################
        # create MJCF model
        ############################
        
        # checkerboard floor
        self._arena = StandardArena()

        # Allegro hand
        self._hand = Hand(
            xml_path= '/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/assets/allegro_hand_v5/urdf/allegro_hand_v5_leo.xml',
            #xml_path= '/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/assets/allegro_hand_v5/urdf/allegro_hand_v5_right.xml',
            #xml_path= '/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/assets/allegro_hand_v4/urdf/allegro_hand_v4_right.xml',
            joint_names = [f"joint_{i}_0" for i in range(16)],
            actuator_names = [f"motor_{i}_0" for i in range(16)]
        )

        # attach hand to arena
        self._arena.attach(
            self._hand.mjcf_model, pos=[0,0,0.15], quat=[0.7071068, 0, 0, -0.7071068]
        )

        # attach box to arena
        # 1. Define your parameters
        m = 2.0      # Mass in kg
        mu = 0.7     # Sliding friction coefficient
        size = 0.1   # Half-length of the cube (Total size: 20cm x 20cm x 20cm)

        # 2. Create the MJCF XML string
        # We inject the Python variables into the XML using an f-string
        box_xml = f"""
            <mujoco>
            <worldbody>
                <body name="my_cube">
                <geom type="box" 
                        size="{size} {size} {size}" 
                        mass="{m}" 
                        friction="{mu} 0.005 0.0001" 
                        rgba="0.8 0.2 0.2 1"/>
                </body>
            </worldbody>
            </mujoco>
            """
        box_model = mjcf.from_xml_string(box_xml)
        self._arena.attach_free(
            box_model, pos=[0,0.2,0.35], quat=[0.7071068, 0, 0, -0.7071068]
        )

        # generate model
        self._physics = mjcf.Physics.from_mjcf_model(self._arena.mjcf_model)

        # set up controller
        self._controller = JointEffortController(
            physics=self._physics,
            joints=self._hand.joints,
            min_effort=-150.0,
            max_effort=150.0
        )

        # for GUI and time keeping
        self._timestep = self._physics.model.opt.timestep
        self._viewer = None
        self._step_start = None

    def _get_obs(self) -> np.ndarray:
        # TODO come up with an observations that makes sense for your RL task
        return np.zeros(6)

    def _get_info(self) -> dict:
        # TODO come up with an info dict that makes sense for your RL task
        return {}

    def reset(self, seed=None, options=None) -> tuple:
        super().reset(seed=seed)

        # reset physics
        with self._physics.reset_context():
            # put arm in a reasonable starting position
            self._physics.bind(self._hand.joints).qpos = [0.1 for _ in range(len(self._hand.joints)) ]

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def step(self, action: np.ndarray) -> tuple:
        # TODO use the action to control the arm

        # run controller to move to target pose
        tau = [0 for _ in range(len(self._hand.joints)) ]
        self._controller.run(tau)

        # step physics
        self._physics.step()

        # render frame
        if self._render_mode == "human":
            self._render_frame()
        
        # TODO come up with a reward, termination function that makes sense for your RL task
        observation = self._get_obs()
        reward = 0
        terminated = False
        info = self._get_info()

        return observation, reward, terminated, False, info

    def render(self) -> np.ndarray:
        """
        Renders the current frame and returns it as an RGB array if the render mode is set to "rgb_array".

        Returns:
            np.ndarray: RGB array of the current frame.
        """
        if self._render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self) -> None:
        """
        Renders the current frame and updates the viewer if the render mode is set to "human".
        """
        if self._viewer is None and self._render_mode == "human":
            # launch viewer
            self._viewer = mujoco.viewer.launch_passive(
                self._physics.model.ptr,
                self._physics.data.ptr,
            )
        if self._step_start is None and self._render_mode == "human":
            # initialize step timer
            self._step_start = time.time()

        if self._render_mode == "human":
            # render viewer
            self._viewer.sync()

            # TODO come up with a better frame rate keeping strategy
            time_until_next_step = self._timestep - (time.time() - self._step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            self._step_start = time.time()

        else:  # rgb_array
            return self._physics.render()

    def close(self) -> None:
        """
        Closes the viewer if it's open.
        """
        if self._viewer is not None:
            self._viewer.close()