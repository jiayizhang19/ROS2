from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


####################################################################################################################################
#Example how to run the launch command
#>ros2 launch robot_bringup rx200_control_only.launch.py xg:=0.40 yg:=-0.3 zg:=0.02 color_seq:='yellow, red, blue'
####################################################################################################################################

def generate_launch_description():
    ld = LaunchDescription()

    color_seq = DeclareLaunchArgument('color_seq', default_value='', description='Customed color sequence of cube stacking')

    goal_x = DeclareLaunchArgument('xg', default_value='0.0', description='x coordinate goal postion')
    goal_y = DeclareLaunchArgument('yg', default_value='0.4', description='y coordinate goal postion')
    goal_z = DeclareLaunchArgument('zg', default_value='0.0', description='z coordinate goal postion')

    linear_move = DeclareLaunchArgument('linear', default_value='False', description='Linear Movement on long distance')

    stack_manager = Node(
        package='rx200_moveit_control',
        executable='stack_manager',
        name='stack_manager',
        parameters=[{
            'color_seq': LaunchConfiguration('color_seq')
        }]
    )

    safety_checker = Node(
        package='rx200_moveit_control',
        executable='point_safety_checker',
        name='point_safety_checker',
    )

    moveit_control = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_client',
        name='rx200_moveit_control',
        parameters=[{
            'linear': LaunchConfiguration('linear'),

            'xg': LaunchConfiguration('xg'),
            'yg': LaunchConfiguration('yg'),
            'zg': LaunchConfiguration('zg')
        }]
    )
    # Linear Movement
    ld.add_action(linear_move)

    # Send color sequence
    ld.add_action(color_seq)

    # Send the goal position coordinate
    ld.add_action(goal_x)
    ld.add_action(goal_y)
    ld.add_action(goal_z)
    
    ld.add_action(stack_manager)
    # ld.add_action(safety_checker)
    ld.add_action(moveit_control)

    return ld