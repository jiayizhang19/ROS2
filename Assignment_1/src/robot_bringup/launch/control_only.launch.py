# Minimal launch file example -- here we are not launching anything
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Commands to run this launch file:
# build and source your workspace first then run:
# ros2 launch robot_bringup control_only.launch.py

def generate_launch_description():
    '''Ensure you have the '''
    ld = LaunchDescription()

    dc = DeclareLaunchArgument(
        'default_gr_state',
        default_value='True',
        description='What should the state of the gripper be',
    )

    moveit_control = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_client',
        parameters=[{
            'start_state_gripper': LaunchConfiguration('default_gr_state'),
        }]

    )       

    # Create you node actions using the Node object
    # Add nodes using ld.add_action

    ld.add_action(dc)
    ld.add_action(moveit_control)
    

    return ld