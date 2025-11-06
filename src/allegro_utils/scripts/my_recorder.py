#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import threading
import sys

class AllegroRecorder(Node):
    def __init__(self, csv_file='allegro_log.csv'):
        super().__init__('allegro_recorder')
        self.csv_file = csv_file
        self.joint_state = None
        self.lock = threading.Lock()

        # Subscriber to joint states
        self.subscription = self.create_subscription(
            JointState,
            'allegroHand_0/joint_states',
            self.joint_state_callback,
            10
        )

        # Start keyboard listener in a separate thread
        thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        thread.start()

        # Open CSV and write header if file does not exist
        try:
            with open(self.csv_file, 'x', newline='') as f:
                writer = csv.writer(f)
                header = []
                # We'll wait until we have a joint_state to write header
                while self.joint_state is None:
                    rclpy.spin_once(self, timeout_sec=0.1)
                n = len(self.joint_state.name)
                header = []
                # positions
                for i in range(n):
                    header.append(f'{self.joint_state.name[i]}_pos')
                # velocities
                for i in range(n):
                    header.append(f'{self.joint_state.name[i]}_vel')
                # efforts
                for i in range(n):
                    header.append(f'{self.joint_state.name[i]}_effort')
                writer.writerow(header)
        except FileExistsError:
            pass

        self.get_logger().info("Allegro Recorder Node Started. Press Enter to log current joint state.")

    def joint_state_callback(self, msg):
        with self.lock:
            self.joint_state = msg


    def keyboard_listener(self):
        while True:
            input("Press Enter to log current joint state...")
            with self.lock:
                if self.joint_state is None:
                    self.get_logger().warn("No joint state received yet! Wait a bit...")
                    continue  # skip logging
                # Make sure positions, velocities, efforts are not empty
                if len(self.joint_state.position) == 0:
                    self.get_logger().warn("Joint state is empty. Skipping...")
                    continue
                row = []
                row += self.joint_state.position
                row += self.joint_state.velocity
                row += self.joint_state.effort

                with open(self.csv_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(row)
                self.get_logger().info("Joint state logged!")

            

def main(args=None):
    rclpy.init(args=args)
    node = AllegroRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down Allegro Recorder Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
