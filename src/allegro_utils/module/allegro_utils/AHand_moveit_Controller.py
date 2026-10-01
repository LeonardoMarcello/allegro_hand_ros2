import rclpy

# TO DO
import moveit_commander

import yaml

import sys
import actionlib

from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
from moveit_msgs.msg import DisplayTrajectory, CollisionObject
from std_msgs.msg import Int32, Header
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from qb_device_srvs.srv import Trigger, GetMeasurements, SetCommands

from dynamic_reconfigure.parameter_generator_catkin import *
from dynamic_reconfigure.server import Server
#from temperature_ros.config import demo_tactip_cfgConfig


class RobotController(object):
    # Class used to manage the robot movement    
    def __init__(self, end_effector:str = 'panda_link8', control_points:str = None):
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            self.robot_group_name = "panda_arm"
            self.end_effector = end_effector
            self.robot.get_group(self.robot_group_name).set_end_effector_link(end_effector)
        
            self.error_recovery = actionlib.SimpleActionClient('/franka_control/error_recovery', ErrorRecoveryAction)
            self.error_recovery.wait_for_server()
            #self.srv = Server(demo_tactip_cfgConfig, self.dyn_rec_callback)
        except Exception as e:
            rospy.logerr(e)
        #self.error_recovery = actionlib.SimpleActionClient('/franka_control/error_recovery', ErrorRecoveryAction)
        #self.error_recovery.wait_for_server()

        self.joint_control_points: dict = {}
        if control_points is not None:
            self.load_control_point_as_yaml(control_points, override=True)


        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20,
        )

        # Add Collision
        count_publisher = rospy.Publisher("/collision_object_count", Int32, queue_size=10) 
        self.collision_pub = rospy.Publisher("/collision_object", CollisionObject, queue_size=10)
        rospy.sleep(1)
        all_points = []
        count_publisher.publish(len(all_points))

        file_name = "ambiente"  # Puoi cambiare il nome del file come preferisci    
        self.add_environment_collision_objects(file_name)

        # Grasp
        #self.activate_motors_client = rospy.ServiceProxy('/communication_handler/activate_motors', Trigger)
        #self.deactivate_motors_client = rospy.ServiceProxy('/communication_handler/deactivate_motors', Trigger)
        #self.get_measurements_client = rospy.ServiceProxy('/communication_handler/get_measurements', GetMeasurements)
        #self.set_commands_client = rospy.ServiceProxy('/communication_handler/set_commands', SetCommands)
        # topic
        self.qb_hand_pub = rospy.Publisher("/qbhand1/control/qbhand1_synergy_trajectory_controller/command", JointTrajectory, queue_size=10) 

    # ------------------------------------------------------------------------------
    #           Routine
    # ------------------------------------------------------------------------------

    # ----------------------
    #   State
    # ----------------------
    def get_current_config(self):
        # Get current robot configuration 
        move_group = self.robot.get_group(self.robot_group_name)
        config = move_group.get_current_joint_values()
        return config
    
    # ----------------------
    #   Movement
    # ----------------------
    def go_to(self, target_pose, type='joint'):
        # Execute planned movement
        move_group = self.robot.get_group(self.robot_group_name)
        if type == 'joint':
            move_group.go(target_pose, wait=True)
        else:
            move_group.set_pose_target(target_pose)        
            
            plan = False
            attempt = 1
            MAX_ATTEMPTS = 3
            while not plan and attempt <= MAX_ATTEMPTS:
                plan = move_group.plan()              
                plan = move_group.go()
                stop = False
                if plan:
                    move_group.stop()
                    move_group.clear_pose_targets()
                else:
                    print("Pose not reached ")
                    stop = True
                    goal = ErrorRecoveryActionGoal()
                    goal.header = Header()

                    self.error_recovery.send_goal(goal)
                    wait = self.error_recovery.wait_for_result(rospy.Duration(5.0))
                    if not wait:
                        rospy.logerr("ErrorRecoveryActionGoal Action server not available!")
                print("Attempt N: ", attempt)
                attempt = attempt + 1  	      
                
            return plan, stop
        
    def back_to_ready(self):
        move_group = self.robot.get_group(self.robot_group_name)
        move_group.set_named_target('ready')
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    def add_current_control_point(self, name):
        # Add current Robot configuration in possible target pose dictionary
        config = self.get_current_config()
        self.joint_control_points[name]=config
        return config
    
    def print_control_point(self):
        # Print target pose dictionary
        for name,pose in self.joint_control_points.items():
            print(f" '{name}':")
            for q in pose:
                print(f"\t{q}")
    
    def load_control_point_as_yaml(self, path, override = False):
        # Read control points from yaml file
        with open(path, 'r') as file:
            data = yaml.safe_load(file)        
        if override:
            self.joint_control_points = data
        else:
            self.joint_control_points.update(data)

    def save_control_point_as_yaml(self, name):
        # Save to a YAML file
        with open(name, 'w') as file:
            yaml.dump(self.joint_control_points, file, sort_keys=False)

    # ----------------------
    #   Collision
    # ----------------------  
    def add_collision_object(self, center, dimensions, object_id, shape_type="BOX"):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = object_id
        primitive = SolidPrimitive()
        if shape_type == "BOX":
            primitive.type = SolidPrimitive.BOX
        elif shape_type == "PLANE":
            primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [dimensions[0], dimensions[1], dimensions[2]]
        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = center[2]
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        self.collision_pub.publish(collision_object)

    def remove_collision_object(self, object_id):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE
        self.collision_pub.publish(collision_object)

    def add_environment_collision_objects(self,file_name):
        base_center = [0.0, 0.0, 0.0]
        base_dimensions = [2.0, 1.0, 0.02]
        self.add_collision_object(base_center, base_dimensions, f"{file_name}_base", shape_type="PLANE")
        
        base_center_2 = [-0.3, 0.0, 0.5]
        base_dimensions_2 = [0.05, 1.0, 1.5]
        self.add_collision_object(base_center_2, base_dimensions_2, f"{file_name}_muro_dietro", shape_type="PLANE")
        
        base_center_3 = [0.0, 0.2, 0.3]
        base_dimensions_3 = [0.1, 0.1, 0.5]
        self.add_collision_object(base_center_3, base_dimensions_3, f"{file_name}_camera_1", shape_type="PLANE")
        
        base_center_4 = [1.2, -0.33, 0.3]
        base_dimensions_4 = [0.1, 0.10, 0.5]
        self.add_collision_object(base_center_4, base_dimensions_4, f"{file_name}_camera_2", shape_type="PLANE")

        base_center_5 = [0.0, -0.5, 0.3]
        base_dimensions_5 = [1, 0.01, 1]
        self.add_collision_object(base_center_5, base_dimensions_5, f"{file_name}_muro_sinistro", shape_type="PLANE")

        base_center_6 = [-0.10, 0.25, 0.3]
        base_dimensions_6 = [0.15, 0.15, 0.5]
        self.add_collision_object(base_center_6, base_dimensions_6, f"{file_name}_cubo_destro", shape_type="PLANE")

        base_center_7 = [0.0, 0.43, 0.3]
        base_dimensions_7 = [1, 0.01, 1]
        self.add_collision_object(base_center_7, base_dimensions_7, f"{file_name}_muro_destro", shape_type="PLANE")


    # ----------------------
    #   Grasp
    # ----------------------
    def grasp(self, percentage = 0.5):
        if percentage<0 or percentage>1:
            rospy.logwarn('[WARN] closure percentage out of range. It will be saturated to 0 or 1')
        saturated_percentage = max(0,min(percentage,1))


        # Close SoftHand
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [saturated_percentage]
        trajectory_point.time_from_start.secs = 1
        #trajectory_point.velocities = [0.5]
        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = ""
        cmd.joint_names = ["qbhand1_synergy_joint"]
        cmd.points = [trajectory_point]
        
        # QB Hand services
        #set_commands_srv = SetCommands()
        #set_commands_srv.id = self.id
        #set_commands_srv.max_repeat = self.max_repeat
        #set_commands_srv.set_commands
        #set_commands_srv.set_commands_async
        #set_commands_srv.commands

        #response = self.set_commands_client(set_commands_srv)
        #response.success
        #response.failures
        self.qb_hand_pub.publish(cmd)

    def release(self):        
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [0.0]
        trajectory_point.time_from_start.secs = 1
        #trajectory_point.velocities = [0.5]
        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = ""
        cmd.joint_names = ["qbhand1_synergy_joint"]
        cmd.points = [trajectory_point]

        # Open SoftHand
        self.qb_hand_pub.publish(cmd)
    