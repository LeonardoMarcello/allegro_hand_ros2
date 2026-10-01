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


def generate_launch_description():

    # Packages
    bringup_pkg_share = get_package_share_directory('allegro_hand_bringup')
    controllers_pkg_share = get_package_share_directory('allegro_hand_controllers')


    # include files
    # ros2 launch allegro_hand_controllers allegro_hand_sim.launch.py HAND:=right TYPE:=B VISUALIZE:=true
    hand = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(controllers_pkg_share,'launch','allegro_hand.launch.py')]
                ),
                launch_arguments={'HAND': 'right', 
                                  'TYPE':'B',
                                  'GUI':'true',
                                  'VISUALIZE':'false',}.items()
    )


    # ros2 bag record -o <bhand_controller> /allegroHand_0/joint_states /allegroHand_0/joint_cmd
    # ros2 bag record -o <leo_controller> /allegroHand_0/joint_states /allegro_effort_controller/command


    # Execute process
    return LaunchDescription([
        # Launch robot
        hand,
    ])
