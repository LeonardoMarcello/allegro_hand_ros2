import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Check if we're told to use sim time
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_share = get_package_share_directory('allegro_hand_bringup')

    # Path to your URDF or xacro
    xacro_file = os.path.join(pkg_share, 'config', 'allegro_hand_ros2control.xacro')
    #robot_description_config = xacro.process_file(xacro_file).toxml() # <-- comment
    robot_description_config = Command(['xacro ', xacro_file, ' ros2_control_hardware_type:=', ros2_control_hardware_type]) # <- ORI


    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        output='screen',
        executable='robot_state_publisher',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            "ros2_control_hardware_type",
            default_value="physical_device",
            description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, physical_device, gazebo, isaac]"),


        node_robot_state_publisher
    ])