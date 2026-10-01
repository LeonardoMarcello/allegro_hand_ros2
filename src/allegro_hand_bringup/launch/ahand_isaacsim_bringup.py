from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler,IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os
import getpass


from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
from functools import partial

## TO DO ##

def generate_launch_description():
    # Argument
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type",default="physical_device")

    # Packages
    pkg_share = get_package_share_directory('allegro_hand_bringup')
    description_pkg_share = get_package_share_directory('allegro_hand_description')
    hw_pkg_share = get_package_share_directory('allegro_hand_hw_interface')
    isaac_pkg_share = get_package_share_directory('allegro_hand_isaacsim')

    # Path to your URDF or ros2 xacro
    xacro_file = os.path.join(pkg_share, 'config', 'allegro_hand_ros2control.xacro')
    robot_description = Command([
        'xacro ', xacro_file,' ros2_control_hardware_type:=mock_components'])


    # Controller manager params file (optional)
    config_path = os.path.join(pkg_share, 'config', 'allegro_controllers.yaml')



    # robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_share,'launch','rsp.launch.py')]
                ),
                launch_arguments={'use_sim_time': 'false', 'ros2_control_hardware_type':'mock_components'}.items()
    )


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

    # joint broadcaster node (publish /joint_state)
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_node,
            on_start=[joint_broad_spawner],
        )
    )

    # Effort controller node (apply torqu on hw)
    effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["allegro_effort_controller"],
        #arguments=["allegro_position_controller"],
    )
    delayed_effort_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_node,
            on_start=[effort_controller],
        )
    )

    # Execute process
    return LaunchDescription([

        # Declare ros2_control_hardware_type argument
        DeclareLaunchArgument(
            "ros2_control_hardware_type",
            default_value="physical_device",
            description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, physical_device, gazebo, isaac]"),


        # Launch robot state publisher
        rsp,

        ## Load the controller manager with your hardware interface
        controller_node,

        ## Delay the joint broadcaster until the controller manager is up
        delayed_joint_broad_spawner,

        ## Delay the effort controller until the controller manager is up
        delayed_effort_controller,

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
