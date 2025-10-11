from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'allegro_hand_hw_interface'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to your URDF or xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'allegro_hand_ros2control.xacro')
    robot_description = Command(['xacro ',xacro_file])

    # Controller manager params file (optional)
    config_path = os.path.join(pkg_share, 'config', 'allegro_controllers.yaml')

    # controller node
    controller_node =   Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }, config_path],
            output='screen'
    )

    # Execute process

    return LaunchDescription([

        ## Load the controller manager with your hardware interface
        controller_node,

        ## Spawn the joint_state_broadcaster
        #ExecuteProcess(
        #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        #    output='screen'
        #),
#
        ## Spawn the effort controller (replace with your controller)
        #ExecuteProcess(
        #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'allegro_effort_controller'],
        #    output='screen'
        #),
    ])
