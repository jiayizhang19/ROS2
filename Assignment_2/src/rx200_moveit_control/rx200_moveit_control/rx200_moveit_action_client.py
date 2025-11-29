#!/usr/bin/env python3
import os
import yaml
import rclpy
import time
import math
import sys
import numpy as np
import tf_transformations
from scipy.spatial.transform import Rotation as Rota
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint, DisplayTrajectory
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion,Point
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String
import json

#####################################################################################################################
#How to run me from command prompt
#>ros2 launch robot_bringup control_only.launch.py xs:=0.25 ys:=0.0 zs:=0.02 xg:=0.40 yg:=-0.3 zg:=0.02 linear:=False
#####################################################################################################################

class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')
        self.group_name_arm = 'interbotix_arm'
        self.group_name_gripper = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'
               
        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for Action Server')

        self.get_logger().info('MoveItEEClient Node initialized successfully!')

        self.create_subscription(
            String,
            '/stack_sequence',
            self.stack_sequence_callback,
            10
        )
        
        self.point_input_pub = self.create_publisher(
            Point, 
            '/point_input', 
            10        
        )
        self.get_logger().info('Point Input Pub Node initialized successfully')

        self.safety_check_sub = self.create_subscription(
            Bool,
            '/point_safety_check',
            self.point_safety_check_callback,
            10
        )
        
        
      
        self.declare_parameter('linear', False)
        self.declare_parameter('xg', 0.35)
        self.declare_parameter('yg', 0.00)
        self.declare_parameter('zg', 0.05)

        self.linear_move_enabled = self.get_parameter('linear').value
        self.x_goal = self.get_parameter('xg').value
        self.y_goal = self.get_parameter('yg').value
        self.z_goal = self.get_parameter('zg').value

        self.yaml_dir = os.path.join(
             get_package_share_directory('rx200_moveit_control'),
             'config',
         )        
        self.poses = self.load_from_yaml(self.yaml_dir, 'poses.yaml')

        self.verbose = (self.poses['debug']['verbose'])
        
        # self.start_point = (self.x_start, self.y_start, self.z_start)
        self.goal_point = (self.x_goal, self.y_goal, self.z_goal)

        # #Future Check for Start and Goal Points will be done with a separate Node.
        # point_msg = Point()
        # point_msg.x = self.x_start 
        # point_msg.y = self.y_start
        # point_msg.z = self.z_start
        # self.point_input_pub.publish(point_msg)

        # point_msg = Point()
        # point_msg.x = self.x_goal
        # point_msg.y = self.y_goal
        # point_msg.z = self.z_goal
        # self.point_input_pub.publish(point_msg)

        # self.get_logger().info(f'Start point: {self.start_point}') 
        self.get_logger().info(f'Goal point: {self.goal_point}') 
        
        self.z_up = self.poses['z_axis_up']
        self.safe_pose = (
            self.poses['safe_pose']['x'],
            self.poses['safe_pose']['y'],
            self.poses['safe_pose']['z'],
        )
        self.safe_pose_lower = (
            self.poses['safe_pose']['x'],
            self.poses['safe_pose']['y'],
            self.poses['safe_pose']['z_lower'],
        )           
        
        self.linear_move_steps = self.poses['linear_move']['steps']
        self.start_gripper_state = self.poses['gripper']['start_gripper_state']
         
        if self.verbose: self.get_logger().info(f'x_goal: {self.x_goal}') 
        if self.verbose: self.get_logger().info(f'y_goal: {self.y_goal}') 
        if self.verbose: self.get_logger().info(f'z_goal: {self.z_goal}')                        
        
        #Default Base radius 
        self.base_radius = self.poses['no_go_zone']['base_radius']
        #Default Out reach circunsference
        self.out_radius = self.poses['no_go_zone']['out_radius']
        #Default Base Box (rectangle) Definition
        self.x1 = self.poses['no_go_zone']['x1']
        self.x2 = self.poses['no_go_zone']['x2']
        self.y1 = self.poses['no_go_zone']['y1']
        self.y2 = self.poses['no_go_zone']['y2']
        #Safe Z hight to move on top of base circle or base
        self.z_hieght_safe = 0.15
        
        #Default radius to decide on pitch = 0.0 or pitch = 1.57
        self.radius_pitch = self.poses['pitch_area']['radius_pitch']
        
        #Default Base Box area to define pitch = 0.0 or pitch 1.57
        self.x1_pitch = self.poses['pitch_area']['x1_pitch']
        self.x2_pitch = self.poses['pitch_area']['x2_pitch']
        self.y1_pitch = self.poses['pitch_area']['y1_pitch']
        self.y2_pitch = self.poses['pitch_area']['y2_pitch']
                
        #pitch angles
        self.horizontal_pitch = self.poses['pitch_angles']['horizontal_pitch']
        self.vertical_pitch = self.poses['pitch_angles']['vertical_pitch']
        self.home_pitch = self.poses['pitch_angles']['home_pitch']
            
        #Check Pick up and drop off location. Make sure not inside the base circle or box
        if self.base_circle_box_inside(self.x_start, self.y_start, self.z_start):
            self.get_logger().info(f'Unsafe pick up location inside arm base x:{self.x_start} y:{self.y_start} z:{self.z_start}')
            rclpy.shutdown()
            sys.exit(0)
        if self.base_circle_box_inside(self.x_goal, self.y_goal, self.z_goal):
            self.get_logger().info(f'Unsafe drop off location inside arm base x:{self.x_goal} y:{self.y_goal} z:{self.z_goal}') 
            rclpy.shutdown()
            sys.exit(0)
        #Check Pick up and drop off location. Make sure not beyond arm's reach    
        if self.out_arm_reach_area(self.x_start, self.y_start, self.z_start):
            self.get_logger().info(f'Too far pick up location beyond arm reach x:{self.x_start} y:{self.y_start} z:{self.z_start}')
            rclpy.shutdown()
            sys.exit(0)
        if self.out_arm_reach_area(self.x_goal, self.y_goal, self.z_goal):
            self.get_logger().info(f'Too far drop off location beyond arm reach x:{self.x_goal} y:{self.y_goal} z:{self.z_goal}') 
            rclpy.shutdown()
            sys.exit(0)
    
    #Simple Function to try communication between Nodes
    def point_safety_check_callback(self, msg: Bool):
        self.get_logger().info(f'Received safety check result: {msg.data}')
        self.is_safety_check_passed = msg.data
    

    def stack_sequence_callback(self, msg: String):
        stack_seq = self.get_cubes(msg)
        if not self.goal_point:
            
            self.goal_point = stack_seq[0]


    def get_cubes(self, msg: String):
        stack_seq = json.loads(msg.data)
        self.get_logger().info(f'Receving stack sequence from stack manager: {stack_seq}')


        return stack_seq



    def load_from_yaml(self, dir, file):
        path = os.path.join(dir, file)
        print(f'Loading poses from: {path}')
        with open(path, 'r') as file:
            return yaml.safe_load(file)

    def movement_strategy(self):
        if self.linear_move_enabled:
            self.linear_move(self.linear_move_steps)
        else:
            self.free_move()

    def free_move(self):
        start_gripper_state = self.start_gripper_state
        self.get_logger().info('Reset to Home position')        
        self.send_ee_pose(*self.safe_pose_lower, True)  # move to the safe place first
        self.get_logger().info('Opening Gripper')
        self.send_gr_pose(start_gripper_state)
        self.get_logger().info('Moving to pick object at Start Point')
        self.send_ee_pose(self.x_start, self.y_start, self.z_start, False)  # lower to start pose
        self.get_logger().info('Closing Gripper')
        self.send_gr_pose(open=False)  # close gripper
        self.send_ee_pose(self.x_start, self.y_start, self.z_start + self.z_up, False)  # move above start pose        
        self.send_ee_pose(self.x_goal, self.y_goal, self.z_goal + self.z_up, False)  # move above target pose  
        time.sleep(3.0) # This move wehn in quadrant (-x, -y) needs more time for some reason!!
        self.get_logger().info('Moving to drop object at Goal Point')
        self.send_ee_pose(self.x_goal, self.y_goal, self.z_goal, False)  # move to target pose
        self.get_logger().info('Opening Gripper')
        self.send_gr_pose(start_gripper_state)  # open gripper      
        self.get_logger().info('Returning to Home position') 
        time.sleep(4.0) 
        self.send_ee_pose(self.x_goal, self.y_goal, self.z_goal + self.z_up, False)  # move above target pose  
        self.send_ee_pose(*self.safe_pose, True)  # move to the safe place
        time.sleep(3.0)
        self.send_ee_pose(*self.safe_pose_lower, True)  # move to the safe place
    
    def linear_move(self, steps):        
        #start_gripper_state = self.start_gripper_state
        z_start_up = self.z_start + self.z_up
        z_goal_up = self.z_goal + self.z_up
        waypoints = self.linear_waypoints(
            (self.x_start, self.y_start, z_start_up),
            (self.x_goal, self.y_goal, z_goal_up),
            steps
        )
        self.get_logger().info('Reset to Home position')        
        self.send_ee_pose(*self.safe_pose_lower, True)  # Start from a safe pose
        self.get_logger().info('Opening Gripper')
        self.send_gr_pose(self.start_gripper_state)
        self.get_logger().info('Moving to pick object at Start Point')
        self.send_ee_pose(self.x_start, self.y_start, self.z_start,False)  # Move to the start pose
        self.get_logger().info('Closing Gripper')
        self.send_gr_pose(False)  # Close gripper
        self.send_ee_pose(self.x_start, self.y_start, z_start_up, False) # Move above the start pose
        for waypoint in waypoints:
            x, y, z = waypoint
            if self.verbose: self.get_logger().info(f'Moving to: ({x:.3f} {y:.3f} {z_start_up:.3f})')         
            self.send_ee_pose(x, y, z, False) 
        self.get_logger().info('Moving to drop object at Goal Point')
        self.send_ee_pose(self.x_goal, self.y_goal, self.z_goal, False)  # Lower to the goal pose
        self.get_logger().info('Opening Gripper')
        self.send_gr_pose(self.start_gripper_state)  # Open gripper
        #self.send_ee_pose(x_goal, y_goal, z_goal_up, False)  # Move above the goal pose
        self.get_logger().info('Returning to Home position') 
        time.sleep(4.0)
        self.send_ee_pose(*self.safe_pose, True)  # move to the safe place
        time.sleep(3.0)
        self.send_ee_pose(*self.safe_pose_lower, True)  # move to the safe place

    def linear_waypoints(self, start, goal, steps):
        waypoints = []
        for step in range(steps + 1):
            increment = step / steps      
            x = start[0] + increment * (goal[0] - start[0])
            y = start[1] + increment * (goal[1] - start[1])
            z = start[2] + increment * (goal[2] - start[2])
            waypoints.append((x, y, z))
        return waypoints 
        
    #This checks are True if coordinates are inside the base areas and False of outside the areas.   
    def base_circle_box_inside(self, x, y, z):        
        point_radius = math.sqrt(x**2 + y**2)        
        base_circle_check = point_radius <= self.base_radius
        
        if x <= self.x1 and x >= self.x2 and y <= self.y2 and y >= self.y1:  
            base_box_check = True
        else:
            base_box_check = False
        #base_box_check = (x <= self.x1) and (x >= self.x2) and (y <= self.y1) and (y <= self.y2 ) 
        z_hieght_check = z < self.z_hieght_safe  
        if self.verbose: self.get_logger().info(f'Inside Base Circle: {base_circle_check}')
        if self.verbose: self.get_logger().info(f'Inside Base Box: {base_box_check}')
        if self.verbose: self.get_logger().info(f'On Top of Base, Box Base {z_hieght_check}')
        #Any is true if inside the base circle or box
        if (base_circle_check or base_box_check) and z_hieght_check:
            return True
        else:
            return False 

    def out_arm_reach_area(self, x, y, z):
        point_radius = math.sqrt(x**2 + y**2)        
        if point_radius > self.out_radius:
            return True
        else:
            return False

    #This check is use to decide EE pitch if x, y are inside box exclusion area where 0.0 pitch will hit the base
    def base_box_exclusion(self, x, y, z):
        if (x <= self.x1_pitch) and (x >=self.x2_pitch) and (y >= self.y1_pitch) and (y <= self.y2_pitch) and (z < self.z_hieght_safe):
            return True
        else:
            return False 
    
    #This check is use to decide EE pitch if x, y are inside circle exclusion area where 0.0 pitch will hit the base
    def base_circle_exclusion(self, x, y, z):
        pos_radius = math.sqrt(x**2 + y**2)        
        if (pos_radius <= self.radius_pitch) and (z < self.z_hieght_safe):
            return True
        else:
            return False  
            
    def make_pose_toward_point(self, x, y, z, home):
        # Compute yaw so EE points to target in XY
        yaw = np.arctan2(y, x)
        roll = 0.0
        # 90 degree if close to the arms base.
        pitch = self.pitch_rotation(x, y, z, home)
        if self.verbose: self.get_logger().info(f'start pitch: {pitch}')     
        # Convert to quaternion
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        q = q / np.linalg.norm(q)  # normalize to avoid Inverse Kinematics issues 
        return q
    
    # Calculate a circular area from (0,0). If inside use EE in vertical position but if outside use horizontal position
    def pitch_rotation(self, x, y, z, home):
        #pos_radius = math.sqrt(x**2 + y**2)
        insde_circle_exclusion = self.base_circle_exclusion(x,y,z)
        if self.verbose: self.get_logger().info(f'base circle exclusion: {insde_circle_exclusion}')
        inside_box_exclusion = self.base_box_exclusion(x,y,z)
        if self.verbose: self.get_logger().info(f'base box exclusion: {inside_box_exclusion}')
        
        if not insde_circle_exclusion and not inside_box_exclusion and not home:
            return self.horizontal_pitch
        if home:
            return self.home_pitch
        else:
            return self.vertical_pitch
        
    def send_gr_pose(self, open):
        time.sleep(5.0)
        if self.verbose: self.get_logger().info(f'start_gripper_state: {open}') 
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 3.0
        req.num_planning_attempts = 2

        jc =JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.038 if open else 0.01
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        #Original Setting
        goal.planning_options.plan_only = False

        #Change settings to avoid singularities and remove delays.
        #goal.planning_options.plan_only = False 
        #goal.planning_options.replan = True
        #goal.planning_options.replan_attempts = 2
        #goal.planning_options.replan_delay = 3.0

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def send_ee_pose(self, x, y, z, home):
        time.sleep(4.0)
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z         
        q = self.make_pose_toward_point(x, y, z, home)
        if self.verbose: self.get_logger().info(f'Quaternian Values: x:{q[0]:.3f} y:{q[1]:.3f} z:{q[2]:.3f} w:{q[3]:.3f}')
        pose.pose.orientation = Quaternion(
            x=float(q[0]),
            y=float(q[1]), 
            z=float(q[2]), 
            w=float(q[3])
        )        
        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 10

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        #change this value from rx200_moveit_control 0.01 to prevent weird moves
        sp.dimensions = [0.01] # if robot doing weird moves you need to change this value
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link 
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05 
        oc.weight = 1.0  #surpriced if we need to change

        goal_constraint = Constraints()
        goal_constraint.position_constraints = [pc]
        goal_constraint.orientation_constraints = [oc]
        req.goal_constraints = [goal_constraint]

        goal = MoveGroup.Goal()
        goal.request = req
        
        #Original Settings.
        goal.planning_options.plan_only = False # Add innovations in here
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        #Change settings to avoid singularities and remove delays.
        #goal.planning_options.plan_only = False
        #goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10
        goal.planning_options.replan_delay = 3.0
        #goal.planning_options.look_around = True
        #goal.planning_options.look_around_attempts = 10

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )   
            
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.verbose: self.get_logger().error(f'MoveIt goal rejected')
            return
        if self.verbose: self.get_logger().info('MoveIt goal accepted')         
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        if self.verbose: self.get_logger().info(f'[Feedback] {state}')

    def _result_cb(self, future):
        #result = future.result().result
        result_msg = future.result().result
        code = getattr(result_msg.error_code, 'val', 'unknown')
        self.get_logger().info(f'[Result] error_code {code}')
        
        # Publish DisplayTrajectory so RViz can show the path
        #if code == 1:  # SUCCESS        
        #    display_msg = DisplayTrajectory()
        #    display_msg.model_id = self.base_link 
            #display_msg.model_id = "rx200"
        #    display_msg.trajectory_start = result_msg.trajectory_start
        #    display_msg.trajectory.append(result_msg.planned_trajectory)
        #    self.display_pub.publish(display_msg)
        #    self.get_logger().info("Published planned trajectory to /display_planned_path.")
        #else:        
        self.get_logger().info(f'[Result] error_code {code}')

def main():
    rclpy.init()
    node = MoveItEEClient()
    # node.movement_strategy()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()