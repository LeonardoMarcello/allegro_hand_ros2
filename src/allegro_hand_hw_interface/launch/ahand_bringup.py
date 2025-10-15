from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler,IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os
import getpass


from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
from functools import partial


def setup_can(context, xacro_file):
    # --- Generate URDF from xacro ---
    urdf_str = os.popen(f"xacro {xacro_file}").read()

    # --- Parse URDF for the ros2_control hardware params ---
    root = ET.fromstring(urdf_str)
    can_port = "can0"  # default 'can0'
    for ros2_control in root.findall('ros2_control'):
        hw_tag = ros2_control.find('hardware')
        if hw_tag is not None:
            for param in hw_tag.findall('param'):
                if param.attrib.get('name') == 'device':
                    can_port = param.text.strip()
                    break

    commands = [
        f"sudo ip link set {can_port} down",
        #f"sudo ip link set {can_port} type can bitrate 1000000",
        f"sudo ip link set {can_port} up"
    ]

    while True:
        password = getpass.getpass('Enter sudo password: ')
        success = True
        for cmd in commands:
            result = os.system(f'echo "{password}" | sudo -S {cmd}')
            if result != 0:
                print(f"Command failed: {cmd}")
                success = False
                break
        if success:
            print(f'{can_port} setup completed')
            break
        else:
            print(f'{can_port} setup failed. Please try again.')


    return []

def generate_launch_description():
    pkg_name = 'allegro_hand_hw_interface'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to your URDF or xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'allegro_hand_ros2control.xacro')
    robot_description = Command(['xacro ',xacro_file])

    # Controller manager params file (optional)
    config_path = os.path.join(pkg_share, 'config', 'allegro_controllers.yaml')

    # robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(pkg_name),'launch','rsp.launch.py')]
                ),
                launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
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
    )
    delayed_effort_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_node,
            on_start=[effort_controller],
        )
    )

    # get setup_can function handle
    setup_can_func = partial(setup_can, xacro_file=xacro_file)

    # Execute process
    return LaunchDescription([
        # setup can device
        OpaqueFunction(function=setup_can_func),

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
