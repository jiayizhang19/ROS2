#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
import math

class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')

        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for Action Server ...')

        self.group_name_arm= 'interbotix_arm'
        self.group_name_gripper= 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'
        self.safe_pose = (0.15, 0.0, 0.25) # define a safe pose

        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_gripper_state', rclpy.Parameter.Type.BOOL),
                ('x_start', rclpy.Parameter.Type.DOUBLE),
                ('y_start', rclpy.Parameter.Type.DOUBLE),
                ('z_start', rclpy.Parameter.Type.DOUBLE),
                ('x_target', rclpy.Parameter.Type.DOUBLE),
                ('y_target', rclpy.Parameter.Type.DOUBLE),
                ('z_target', rclpy.Parameter.Type.DOUBLE)
            ]
        )   
        
        # self.declare_parameter('start_state_gripper_open', value=True)
        # self.send_gr_pose(self.get_parameter('start_state_gripper_open').value)

        start_pose, target_pose = self.get_start_target_poses()[0], self.get_start_target_poses()[1]

        self.get_logger().info('Node initialized successfully!')
        self.get_logger().info(f'start_pose: {start_pose}, target_pose: {target_pose}')
        self.move(start_pose, target_pose)
    

    def get_start_target_poses(self):
        x_start = self.get_parameter('x_start').value
        y_start = self.get_parameter('y_start').value
        z_start = self.get_parameter('z_start').value
        x_target = self.get_parameter('x_target').value
        y_target = self.get_parameter('y_target').value
        z_target = self.get_parameter('z_target').value

        start = (x_start, y_start, z_start)
        target = (x_target, y_target, z_target)

        return start, target
    

    def send_ee_pose(self, x, y, z, task=None):
        pose = PoseStamped()
        pose.header.frame_id = self.base_link # refenence to base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # no orientation
        # qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 1.57) # 90 degrees yaw
        qx, qy, qz, qw = self.compute_quaternion(target=(x, y, z))
        pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        
        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01] # if robots behave wild, tune this parameter
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05 # tune it in the second assignment
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0 # range [0,1], like weight in machine learning 

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        send_future = self._client.send_goal_async(
            goal, 
            feedback_callback=self._feedback_cb
            )
        send_future.add_done_callback(self._goal_response_cb)

    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.035 if open else 0.0
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False 

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return
        self.get_logger().info('MoveIt goal accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self,feedback_msg):
        state = getattr(feedback_msg.feedback, 'state', '<unknown>')
        self.get_logger().info(f'[Feedback] {state}')

    def _result_cb(self, future, task=None):
        result = future.result().result
        code = getattr(result.error_code, 'val', 'unknown')
        self.get_logger().info(f'[{task} - Result] error_code {code}')

    def move(self, start, target):
        self.send_ee_pose(*self.safe_pose)  # move to a safe place first
        self.send_ee_pose(*start)  # move to start pose
        self.send_gr_pose(open=True)  # open gripper
        self.send_gr_pose(open=False)  # close gripper
        self.send_ee_pose(*self.safe_pose)  # move to a safe place
        self.send_ee_pose(*target)  # move to target pose
        self.send_gr_pose(open=True)  # open gripper
        self.send_ee_pose(*self.safe_pose)  # move to a safe place

    def compute_quaternion(self, target, start=(0.0, 0.0, 0.0)):
        '''
        Compute quaternion from start and target positions in the X-Y plane
        > Outputs: Quaternion (x, y, z, w) in radians
        '''
        x = target[0] - start[0]
        y = target[1] - start[1]
        # compute the angle between two points in the X-Y plane, which is the yaw angle
        yaw = math.atan2(y, x) 
        # convert euler angles to quaternion
        return quaternion_from_euler(0.0, 0.0, yaw)
    


def main():
    rclpy.init()
    node = MoveItEEClient()
    # node.move(
    #     start=(0.25, 0.25, 0.0),
    #     target=(-0.3, -0.3, 0.0)
    # )
    # node.send_ee_pose(-0.3, 0.3, 0.15)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
