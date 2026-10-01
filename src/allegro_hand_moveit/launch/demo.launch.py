from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Retrieve HAND, TYPE and hardware type from the context
    hand = LaunchConfiguration("HAND").perform(context)
    hand_type = LaunchConfiguration("TYPE").perform(context)
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type").perform(context)

    # Dynamically construct the robot name
    robot_name = f"allegro_hand_{hand}_{hand_type}"
    bringup_pkg_share = get_package_share_directory('allegro_hand_bringup') # Leo

    # Build the MoveIt config using the dynamically generated robot name
    moveit_config = MoveItConfigsBuilder(
        robot_name=robot_name, package_name="allegro_hand_moveit"
    ).to_moveit_configs()

    # Define the actions (list of launch elements)
    actions = []

    # Declare boolean launch arguments
    actions.append(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    actions.append(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    actions.append(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # Hand hardware + controllers from bringup (sim: fake hardware, real: position control)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg_share, 'launch', 'ahand_bringup.py')
            ),
            launch_arguments={
                "ros2_control_hardware_type": ros2_control_hardware_type,
                "position_control": "true",   # MoveIt drives the real hand only in position control
                "viz": "false",               # MoveIt has its own RViz
            }.items(),
        )
    )

    # Include move_group.launch.py
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
            launch_arguments={
                "HAND": hand,
                "TYPE": hand_type,
            }.items(),
        )
    )

    # Include moveit_rviz.launch.py (optional)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # Include warehouse_db.launch.py (optional)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Add save_joint_angles_node
    actions.append(
        Node(
            package="allegro_hand_moveit",
            executable="save_joint_angles",
            name="save_joint_angles",
            output="screen",
        )
    )

    return actions


def generate_launch_description():
    # Declare HAND and TYPE as launch arguments
    hand_arg = DeclareLaunchArgument(
        "HAND", default_value="right", description="The hand side (left or right)"
    )
    type_arg = DeclareLaunchArgument(
        "TYPE", default_value="B", description="The hand type (e.g., A, B, C)"
    )
    hw_arg = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="physical_device",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, physical_device, gazebo, isaac]"
    )

    # Create and return the LaunchDescription
    return LaunchDescription([
        hand_arg,
        type_arg,
        hw_arg,
        OpaqueFunction(function=launch_setup),
    ])
