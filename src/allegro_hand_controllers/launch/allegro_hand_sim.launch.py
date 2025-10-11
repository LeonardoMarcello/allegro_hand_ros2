from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os
import getpass

def generate_launch_description():
    allegro_hand_controllers_share = get_package_share_directory('allegro_hand_controllers')
    allegro_hand_moveit_share = get_package_share_directory('allegro_hand_moveit')

    # Declare launch argument
    declare_visualize_arg = DeclareLaunchArgument(
        'VISUALIZE',
        default_value='false',
        description='Flag to enable/disable visualization'
    )
    
    declare_moveit_arg = DeclareLaunchArgument(
        'MOVEIT',
        default_value='false',
        description='Flag to enable/disable moveit2'
    )
	
    declare_gui_arg = DeclareLaunchArgument(
        'GUI',
        default_value='false',
        description='Flag to enable/disable GUI'
    )
	
    declare_sim_arg = DeclareLaunchArgument(
        'ISAAC',
        default_value='false',
        description='Flag to enable/disable ISAAC SIM2REAL'
    )

    declare_hand_arg = DeclareLaunchArgument(
        'HAND',
        default_value='right',
        description='Specify which hand to use: right or left'
    )
    
    declare_type_arg = DeclareLaunchArgument(
        'TYPE',
        default_value='B',
        description='Specify which type to use: A(non-geared) or B(geared)'
    )

    #declare_polling_arg = DeclareLaunchArgument(
    #    'POLLING',
    #    default_value='true',
    #    description='true, false for polling the CAN communication'
    #)
    #
    #declare_can_device_arg = DeclareLaunchArgument(
    #	'CAN_DEVICE', 
    #	default_value='can0',
    #	description='Specify CAN port for control multi devices'
    #)
    
    declare_num_arg = DeclareLaunchArgument(
    	'NUM',
    	default_value='0',
    	description='Specify AH num for control multi devices'
    )
    
       
    urdf_path = PythonExpression([
        '"', allegro_hand_controllers_share, '/urdf/allegro_hand_description_', LaunchConfiguration('HAND'),'_', LaunchConfiguration('TYPE'),'.urdf"'
    ])

    return LaunchDescription([
        declare_visualize_arg,
        #declare_polling_arg,
        #declare_can_device_arg,
        declare_num_arg,
        declare_moveit_arg,
		declare_gui_arg,
		declare_sim_arg,
        #Node(
        #    package='allegro_hand_controllers',
        #    executable='allegro_node_grasp',
        #    output='screen',
        #    parameters=[{'hand_info/which_hand': LaunchConfiguration('HAND')}, # Pass HAND argument to parameter
		#	            {'hand_info/which_type': LaunchConfiguration('TYPE')},
        #    		    {'comm/CAN_CH': LaunchConfiguration('CAN_DEVICE')}],
        #    arguments=[LaunchConfiguration('POLLING')],
		#	remappings=[
		#		('allegroHand/lib_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/lib_cmd'"])),
        #    			('allegroHand/joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"])),
        #            		('allegroHand/joint_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_cmd'"])),
        #            		('allegroHand/envelop_torque',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/envelop_torque'"])),
		#		('allegroHand/tactile_sensors',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/tactile_sensors'"])),
        #            		('forcechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/force_chg'"])),
        #            		('timechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/time_chg'"])),
		#	]
        #),
        #Node(
        #    package='robot_state_publisher',
        #    output='screen',
        #    executable='robot_state_publisher',
        #    parameters=[{'robot_description': Command(['xacro ', urdf_path])},
        #                {'use_sim_time': True}   # << Added this line to use simulation time
        #                ],
        #    
        #    remappings=[
        #        ('tf', PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/tf'"])),
        #        ('joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"])),
        #        ('robot_description', 'allegro_hand_description')
        #    ]
        #),
        # Include the allegro_viz.launch.py file if VISUALIZE is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(allegro_hand_controllers_share, 'launch', 'allegro_viz.launch.py')),
            condition=IfCondition(LaunchConfiguration('VISUALIZE')),
            launch_arguments={'NUM': LaunchConfiguration('NUM'),
                              'use_sim_time': 'true'}.items()
        ),
        # Include the demo.launch.py file if MOVEIT is true       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(allegro_hand_moveit_share, 'launch', 'demo_isaac.launch.py')),
            condition=IfCondition(LaunchConfiguration('MOVEIT')),
            launch_arguments={'HAND': LaunchConfiguration('HAND'),'TYPE': LaunchConfiguration('TYPE')}.items() 
        ),
		Node(
            package='allegro_hand_gui',
            executable='allegro_hand_gui_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('GUI'))
        ),
		Node(
            package='allegro_hand_isaacsim',
            executable='allegro_hand_sim',
            output='screen',
		    remappings=[('allegroHand/joint_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_cmd'"]))],		
            #remappings=[('allegroHand/joint_cmd',PythonExpression(["'/joint_cmd'"]))],		
            condition=IfCondition(LaunchConfiguration('ISAAC')),
            parameters=[{'use_sim_time': True}] # << Added this line to use simulation time
        )
    ])
