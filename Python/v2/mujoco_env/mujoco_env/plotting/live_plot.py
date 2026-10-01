import threading
import queue
import collections
import numpy as np
import matplotlib.pyplot as plt
import sys, os

class LivePlot(threading.Thread):
    def __init__(self, timestep=0.01, update_freq_hz=10, window_sec=10.0):
        """
        Initializes the background plotting thread.
        """
        # Initialize the Thread (daemon=True means it will close when your main script exits)
        super().__init__(daemon=True)

        # A queue of size 1 ensures we only ever plot the most recent data. 
        # If the plotter lags, older frames are dropped rather than accumulating.
        self.data_queue = queue.Queue(maxsize=1) 

        self.is_running = True

        # Configuration
        self._timestep = timestep
        self._plot_window_sec = window_sec
        effective_dt = 1.0 / update_freq_hz
        self.max_pts = int(self._plot_window_sec / effective_dt) + 1

    def update(self, obs, current_time) -> None:
        """
        Called by the MAIN thread to send data to the plotter.
        Does not block the main physics loop.
        """
        # Try to push the latest data. If the queue is full, empty it and push again.
        if self.data_queue.full():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self.data_queue.put_nowait((obs, current_time))
        except queue.Full:
            pass

    def stop_plotting(self):
        """Signals the thread to shut down."""
        self.is_running = False

    def run(self):
        """
        This is the method that runs in the BACKGROUND thread.
        It initializes Matplotlib and loops infinitely to draw data.
        """
        plt.ion() # Enable interactive mode

        # ---------------------------------------------------------
        # 1. Initialize Thunder Hand Model
        # ---------------------------------------------------------
        sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
        import utils.import_thunder as thunder

        index = thunder.thunder_ahand_finger()
        middle = thunder.thunder_ahand_finger()
        ring = thunder.thunder_ahand_finger()
        thumb = thunder.thunder_ahand_thumb()

        # Load config (Update paths as needed)
        base_path = "/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/config/"
        thunder.load_params(index, f"{base_path}ahand_index_identification_par.yaml")
        thunder.load_params(middle, f"{base_path}ahand_middle_identification_par.yaml")
        thunder.load_params(ring, f"{base_path}ahand_ring_identification_par.yaml")
        thunder.load_params(thumb, f"{base_path}ahand_thumb_identification_par.yaml")
        self.fingers = [index, middle, ring, thumb]

        # ---------------------------------------------------------
        # 2. Initialize the Figure and Axes
        # ---------------------------------------------------------
        self._plot_fig = plt.figure(figsize=(14, 10))
        self._plot_fig.canvas.manager.set_window_title("Environment Live Data & Kinematics")

        ax1 = self._plot_fig.add_subplot(2, 2, 1) # Object Pos
        ax2 = self._plot_fig.add_subplot(2, 2, 2) # Contact Forces
        ax3 = self._plot_fig.add_subplot(2, 2, 3) # Joint Mean
        self._ax_3d = self._plot_fig.add_subplot(2, 2, 4, projection='3d') # 3D Kinematics
        self._plot_axes_2d = [ax1, ax2, ax3]

        # Data containers
        self._plot_t = collections.deque(maxlen=self.max_pts)
        self._plot_obj_pos = [collections.deque(maxlen=self.max_pts) for _ in range(3)]
        self._plot_contact_forces = [collections.deque(maxlen=self.max_pts) for _ in range(4)]
        self._plot_joint_q_mean = collections.deque(maxlen=self.max_pts)

        # Setup Lines
        ax1.set_title("Object Position (X, Y, Z)")
        ax1.set_ylabel("Meters")
        self._line_obj_x, = ax1.plot([], [], label='X', color='r')
        self._line_obj_y, = ax1.plot([], [], label='Y', color='g')
        self._line_obj_z, = ax1.plot([], [], label='Z', color='b')
        ax1.legend(loc='upper left')

        ax2.set_title("Fingertip Contact Force (Norm)")
        ax2.set_ylabel("Newtons")
        self._line_cf_idx, = ax2.plot([], [], label='Index')
        self._line_cf_mid, = ax2.plot([], [], label='Middle')
        self._line_cf_rng, = ax2.plot([], [], label='Ring')
        self._line_cf_thb, = ax2.plot([], [], label='Thumb')
        ax2.legend(loc='upper left')

        ax3.set_title("Mean Hand Joint Position")
        ax3.set_ylabel("Radians")
        ax3.set_xlabel("Time (s)")
        self._line_q_mean, = ax3.plot([], [], color='purple')

        plt.tight_layout()
        plot_start_time = None

        # ---------------------------------------------------------
        # 3. Main GUI Update Loop
        # ---------------------------------------------------------
        while self.is_running:
            try:
                # Block for 0.05s waiting for data. 
                # If no data arrives, it raises queue.Empty and keeps the GUI responsive.
                obs, current_time = self.data_queue.get(timeout=0.05)

                if plot_start_time is None:
                    plot_start_time = current_time

                t_rel = current_time - plot_start_time
                self._plot_t.append(t_rel)

                # --- Update Allegro Kinematics ---
                q_current = obs['joint_states']['q']
                for f in range(len(self.fingers)):
                    self.fingers[f].set_q(q_current[4*f : 4*(f+1)])

                # --- 2D Data Extraction ---
                obj_x = obs['object_state']['x']
                for i in range(3):
                    self._plot_obj_pos[i].append(obj_x[i])

                self._plot_contact_forces[0].append(np.linalg.norm(obs['contacts']['contact_index']['force-torque'][:3]))
                self._plot_contact_forces[1].append(np.linalg.norm(obs['contacts']['contact_middle']['force-torque'][:3]))
                self._plot_contact_forces[2].append(np.linalg.norm(obs['contacts']['contact_ring']['force-torque'][:3]))
                self._plot_contact_forces[3].append(np.linalg.norm(obs['contacts']['contact_thumb']['force-torque'][:3]))

                self._plot_joint_q_mean.append(np.mean(q_current))

                # --- Update 2D Lines ---
                self._line_obj_x.set_data(self._plot_t, self._plot_obj_pos[0])
                self._line_obj_y.set_data(self._plot_t, self._plot_obj_pos[1])
                self._line_obj_z.set_data(self._plot_t, self._plot_obj_pos[2])

                self._line_cf_idx.set_data(self._plot_t, self._plot_contact_forces[0])
                self._line_cf_mid.set_data(self._plot_t, self._plot_contact_forces[1])
                self._line_cf_rng.set_data(self._plot_t, self._plot_contact_forces[2])
                self._line_cf_thb.set_data(self._plot_t, self._plot_contact_forces[3])

                self._line_q_mean.set_data(self._plot_t, self._plot_joint_q_mean)

                # Shift 2D axes
                x_min = max(0, t_rel - self._plot_window_sec)
                x_max = max(self._plot_window_sec, t_rel)
                for ax in self._plot_axes_2d:
                    ax.set_xlim(x_min, x_max)
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)

                # --- 3D Hand Kinematics Update ---
                self._ax_3d.cla()
                frames = {"WORLD": np.eye(4)}

                for f in range(len(self.fingers)):
                    for i in range(1, self.fingers[f].numJoints+1):
                        Ti = getattr(self.fingers[f], f"get_T_w_{i}")() 
                        frames[f"O{f+1}_{i}"] = Ti
                    frames[f"O{f+1}_EE"] = self.fingers[f].get_T_w_ee()

                for key, T in frames.items():
                    origin = T[:3, 3]
                    x_axis = T[:3, 0] * 0.015
                    y_axis = T[:3, 1] * 0.015
                    z_axis = T[:3, 2] * 0.015
                    self._ax_3d.quiver(*origin, *x_axis, color='r', linewidth=1.5, arrow_length_ratio=.2)
                    self._ax_3d.quiver(*origin, *y_axis, color='g', linewidth=1.5, arrow_length_ratio=.2)
                    self._ax_3d.quiver(*origin, *z_axis, color='b', linewidth=1.5, arrow_length_ratio=.2)

                origins = np.array([T[:3, 3] for T in frames.values()])
                self._ax_3d.plot(origins[:, 0], origins[:, 1], origins[:, 2], 'k--', linewidth=1)

                max_range = (origins.max(axis=0) - origins.min(axis=0)).max() / 2.0
                mid = origins.mean(axis=0)
                self._ax_3d.set_xlim(mid[0] - max_range, mid[0] + max_range)
                self._ax_3d.set_ylim(mid[1] - max_range, mid[1] + max_range)
                self._ax_3d.set_zlim(mid[2] - max_range, mid[2] + max_range)

                self._ax_3d.set_box_aspect([1, 1, 1])
                self._ax_3d.view_init(elev=25, azim=45)

                # Draw the updated plot
                self._plot_fig.canvas.draw()
                self._plot_fig.canvas.flush_events()

            except queue.Empty:
                # If no new data, process GUI events anyway so the window doesn't freeze
                self._plot_fig.canvas.draw_idle()
                self._plot_fig.canvas.flush_events()
            except Exception as e:
                print(f"[Plotting Thread] Error: {e}")
                break

        plt.close(self._plot_fig)