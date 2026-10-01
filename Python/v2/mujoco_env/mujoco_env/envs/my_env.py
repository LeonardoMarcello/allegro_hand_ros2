import time
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

from dm_control import mjcf
import mujoco.viewer
from mujoco_env.arenas import StandardArena
from mujoco_env.robots import Arm, Hand
from mujoco_env.utils.interface import JointEffortInterface

import gymnasium as gym
from gymnasium import spaces


class MyEnv(gym.Env):

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": None,
    }  # TODO add functionality to render_fps

    def __init__(self, render_mode=None):
        # TODO come up with an observation space that makes sense
        # self.observation_space = spaces.Box(
        #     low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64
        # )
        self.observation_space = spaces.Dict({
            'joint_states': spaces.Dict({
                'q': spaces.Box(low=-np.inf, high=np.inf, shape=(16,), dtype=np.float64),
                'dq': spaces.Box(low=-np.inf, high=np.inf, shape=(16,), dtype=np.float64),
                'tau_dyn': spaces.Box(low=-np.inf, high=np.inf, shape=(16,), dtype=np.float64),
            }),
            'object_state': spaces.Dict({
                'x': spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64),
                'quat': spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float64),
                'dx': spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64),
            }),
            'contacts': spaces.Dict({
                    'contact_index': spaces.Dict({
                                        'force-torque' : spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64),
                                        'position' : spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64),
                                        'normal' : spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64)
                                        }),

                    'contact_middle': spaces.Dict({
                                        'force-torque' : spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64),
                                        'position' : spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64),
                                        'normal' : spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64)
                                    }),

                    'contact_ring' :spaces.Dict({
                                        'force-torque' : spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64),
                                        'position' : spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64),
                                        'normal' : spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64)
                                    }),

                    'contact_thumb': spaces.Dict({
                                        'force-torque' : spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64),
                                        'position' : spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64),
                                        'normal' : spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float64)
                                    }),
            })
        })
        # self.action_space = {
        #         'joint_states': {   
        #             'q':spaces.Box(low=-0.1, high=0.1, shape=(6,), dtype=np.float64),
        #             'dq':spaces.Box(low=-0.1, high=0.1, shape=(6,), dtype=np.float64)
        #         }
        # }
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
        # vert hand ------
        self._arena.attach(
            self._hand.mjcf_model, pos=[-0.15,0,0.15], quat=R.from_euler('xyz', [0, 0, 0]).as_quat()#[0.7071068, 0, 0, -0.7071068]
        )
        self._T_hand2world = np.array([[1, 0, 0, -0.15],
                                      [0, 1, 0, 0],
                                      [0, 0, 1, 0.15],
                                      [0, 0, 0, 1],])
        self._T_world2hand = np.linalg.inv(self._T_hand2world)

        # orizz hand --------
        # self._arena.attach(
        #     self._hand.mjcf_model, pos=[0,0,0.15], quat = R.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()
        # )

        # self._T_hand2world = np.array([[0, 0, -1, 0],
        #                               [0, 1, 0, 0],
        #                               [1, 0, 0, 0.15],
        #                               [0, 0, 0, 1],])
        # self._T_world2hand = np.linalg.inv(self._T_hand2world)

        # Finger desired target boint
        self.ee_desired = {}
        for name in ['index','middle','ring','thumb']:
            self.ee_desired[name] = self._arena.mjcf_model.worldbody.add('body', name=f'{name}_target_visual', mocap='true')
            self.ee_desired[name].add('geom', type='sphere', size=[0.01], contype=0, conaffinity=0, 
                            group=1, rgba=[0.2, 0.8, 0.2, 0.5])

        # attach box to arena
        # 1. Define your parameters
        m = 0.05      # Mass in kg
        mu = 0.7      # PUT THIS BACK UP! 0.007 is too slippery and causes sliding jitter.
        size = 0.025   # Total size

        # 2. Create the MJCF XML string
        box_xml = f"""
            <mujoco>
            <worldbody>
                <body name="my_cube">
                <geom type="box" 
                        size="{size} {size} {size}" 
                        mass="{m}" 
                        friction="{mu}" 
                        solref="0.01 1"
                        solimp="0.9 0.95 0.001"
                        rgba="0.8 0.2 0.2 1"/>
                
                <geom pos="{-size} 0 0" type="sphere" size="{size/5}" contype="0" conaffinity="0" rgba="0.8 0.2 0.2 1"/> 
                <geom pos="0 {-size} 0" type="sphere" size="{size/5}" contype="0" conaffinity="0" rgba="0.8 0.8 0.2 1"/>
                <geom pos="{size} 0 0" type="sphere" size="{size/5}" contype="0" conaffinity="0" rgba="0.2 0.8 0.2 1"/>
                <geom pos="0 {size} 0" type="sphere" size="{size/5}" contype="0" conaffinity="0" rgba="0.2 0.2 0.8 1"/>
                </body>
            </worldbody>
            </mujoco>
            """
        box_model = mjcf.from_xml_string(box_xml)
        self._box_body = box_model.find('body', 'my_cube')
        self._arena.attach_free(
            box_model, pos=[0.04,0.0,0.2], quat=[0.7071068, 0, 0, -0.7071068]
        )

        #ft sensor
        ati_xml = f"""
            <mujoco>
                <worldbody>
                    <body name="ft_sensor_base" pos="0 0 0">
                        <geom type="box" 
                              size="{size} {size} {size}" 
                              solref="0.01 1"
                              solimp="0.9 0.95 0.001"
                              rgba="0.0 0.8 0.0 1"/>
                        <site name="sensor_site" pos="0 0 0" size="0.001" rgba="1 0 0 1"/>
                    </body>
                </worldbody>

                <sensor>
                    <force name="ati_force" site="sensor_site"/>
                    <torque name="ati_torque" site="sensor_site"/>
                </sensor>
            </mujoco>
            """
        ati_model = mjcf.from_xml_string(ati_xml)
        #self._arena.attach(ati_model, pos=[0.15, 0.1, 0.2], quat= [0.7071068, 0, 0, -0.7071068])

        # generate model
        self._physics = mjcf.Physics.from_mjcf_model(self._arena.mjcf_model)

        # set up controller
        self._controller = JointEffortInterface(
            physics=self._physics,
            joints=self._hand.joints,
            min_effort=-150.0,
            max_effort=150.0
        )

        # for GUI and time keeping
        self._timestep = self._physics.model.opt.timestep
        self._viewer = None
        self._step_start = None

        current_gravity = self._physics.model.opt.gravity
        print(f"Current gravity vector is: {current_gravity}")

    def _get_obs(self) -> dict:
        obs = {}
        # 1) Hand Configuration
        obs['joint_states'] = {
                'q': self._physics.bind(self._hand.joints).qpos,
                'dq': self._physics.bind(self._hand.joints).qvel,
                'tau_dyn': self._physics.bind(self._hand.joints).qfrc_bias,
            }
        # obs['hand_pose'] = {
        #     "position": self._physics.bind(self._hand.mjcf_model.root_body).xpos,
        #     "quaternion": self._physics.bind(self._hand.mjcf_model.root_body).xquat
        # }
        # 2) Obj
        obs['object_state'] = {
                'x': self._apply_transform(self._physics.bind(self._box_body).xpos),
                'quat': self._physics.bind(self._box_body).xquat,
                'dx': self._physics.bind(self._box_body).cvel,
            }
        # 3) Contacts
        obs['contacts'] = self._get_contact_forces()
        #print(self._physics.bind('contact'))

        return obs

    def _get_info(self) -> dict:
        # TODO come up with an info dict that makes sense for your RL task
        return {}

    def reset(self, seed=None, options=None) -> tuple:
        super().reset(seed=seed)
        home_config = np.array([0.1 for _ in range(len(self._hand.joints)) ]) 
        home_config[12] = np.pi/2
        home_config [4:8]= np.array([0.00614355896702004,0.2964267201587169,0.2303834612632515,0.11519173063162574])
        # reset physics
        with self._physics.reset_context():
            # put arm in a reasonable starting position
            self._physics.bind(self._hand.joints).qpos = home_config

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def step(self, tau: np.ndarray) -> tuple:

        # run controller to apply to target torque
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

    # Utils -----------------------------------------------------------------------------------
    def _apply_transform(self, v, point = True):
        """
        apply transformation to convert world pose into hand frame.
        """
        if point:
            v_ext = np.hstack([v,1])
        else:
            v_ext = np.hstack([v,0])
        v_tranformed = self._T_world2hand @ v_ext
        return v_tranformed[:3]

    def _get_contact_forces(self):
        """
        Returns the forces and position at fingertips contact.
        """
        contacts = {
                    'contact_index':{
                                        'force-torque' : np.zeros(6),
                                        'position' : np.zeros(3),
                                        'normal' : np.zeros(3)
                                    },

                    'contact_middle':{
                                        'force-torque' : np.zeros(6),
                                        'position' : np.zeros(3),
                                        'normal' : np.zeros(3)
                                    },

                    'contact_ring':{
                                        'force-torque' : np.zeros(6),
                                        'position' : np.zeros(3),
                                        'normal' : np.zeros(3)
                                    },

                    'contact_thumb':{
                                        'force-torque' : np.zeros(6),
                                        'position' : np.zeros(3),
                                        'normal' : np.zeros(3)
                                    }
                }

        # 1. Find the geom object inside your hand's MJCF model (ignores prefixes)
        index_geom_obj = self._hand.mjcf_model.find('geom', 'link_3_0_tip')
        index_geom_id = self._physics.bind(index_geom_obj).element_id

        middle_geom_obj = self._hand.mjcf_model.find('geom', 'link_7_0_tip')
        middle_geom_id = self._physics.bind(middle_geom_obj).element_id

        ring_geom_obj = self._hand.mjcf_model.find('geom', 'link_11_0_tip')
        ring_geom_id = self._physics.bind(ring_geom_obj).element_id

        thumb_geom_obj = self._hand.mjcf_model.find('geom', 'link_15_0_tip')
        thumb_geom_id = self._physics.bind(thumb_geom_obj).element_id

        # 2. iterate on contact
        for i in range(self._physics.data.ncon):
            contact = self._physics.data.contact[i]
            if contact.geom1 == index_geom_id or contact.geom2 == index_geom_id:
                contacts['contact_index']['position'] = contact.pos.copy()
                # Extract [Normal, Tangent X, Tangent Y]
                c_array = self._physics.data.contact_force(i)
                contacts['contact_index']['force-torque'] =  c_array[:3].copy()

            elif contact.geom1 == middle_geom_id or contact.geom2 == middle_geom_id:
                contacts['contact_middle']['position'] = contact.pos.copy()
                # Extract [Normal, Tangent X, Tangent Y]
                c_array = self._physics.data.contact_force(i)
                contacts['contact_middle']['force-torque'] =  c_array[:3].copy()
                #break

            elif contact.geom1 == ring_geom_id or contact.geom2 == ring_geom_id:
                contacts['contact_ring']['position'] = contact.pos.copy()
                # Extract [Normal, Tangent X, Tangent Y]
                c_array = self._physics.data.contact_force(i)
                contacts['contact_ring']['force-torque'] =  c_array[:3].copy()
                #break

            elif contact.geom1 == thumb_geom_id or contact.geom2 == thumb_geom_id:
                contacts['contact_thumb']['position'] = contact.pos.copy()
                # Extract [Normal, Tangent X, Tangent Y]
                c_array = self._physics.data.contact_force(i)
                contacts['contact_thumb']['force-torque'] =  c_array[:3].copy()
                #break
        return contacts

    def set_desired_ee(self, name, x_des):
        body_id = self._physics.model.name2id(f'{name}_target_visual', 'body')
        mocap_id = self._physics.model.body_mocapid[body_id]
        self._physics.data.mocap_pos[mocap_id] = x_des

    def get_desired_ee(self, name):
        body_id = self._physics.model.name2id(f'{name}_target_visual', 'body')
        mocap_id = self._physics.model.body_mocapid[body_id]
        return  self._apply_transform(self._physics.data.mocap_pos[mocap_id])
        #return self._physics.data.mocap_pos[mocap_id]
