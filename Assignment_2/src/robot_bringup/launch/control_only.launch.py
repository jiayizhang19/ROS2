from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


####################################################################################################################################
#Example how to run the launch command
#>ros2 launch robot_bringup control_only.launch.py xs:=0.25 ys:=0.0 zs:=0.02 xg:=0.40 yg:=-0.3 zg:=0.02 linear:=False
####################################################################################################################################

def generate_launch_description():
    ld = LaunchDescription()

    start_x = DeclareLaunchArgument('xs', default_value='0.25', description='x coordinate start postion')
    start_y = DeclareLaunchArgument('ys', default_value='0.00', description='y coordinate start postion')
    start_z = DeclareLaunchArgument('zs', default_value='0.05', description='z coordinate start postion')

    goal_x = DeclareLaunchArgument('xg', default_value='0.35', description='x coordinate goal postion')
    goal_y = DeclareLaunchArgument('yg', default_value='0.00', description='y coordinate goal postion')
    goal_z = DeclareLaunchArgument('zg', default_value='0.05', description='z coordinate goal postion')

    linear_move = DeclareLaunchArgument('linear', default_value='False', description='Linear Movement on long distance')

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

            'xs': LaunchConfiguration ('xs'),
            'ys': LaunchConfiguration ('ys'),
            'zs': LaunchConfiguration ('zs'),

            'xg': LaunchConfiguration ('xg'),
            'yg': LaunchConfiguration ('yg'),
            'zg': LaunchConfiguration ('zg')
        }]
    )
    #Linear Movement
    ld.add_action(linear_move)

    #Send the Start Point Coordinate
    ld.add_action(start_x)
    ld.add_action(start_y)
    ld.add_action(start_z)

    #Send the End Point Coordinate
    ld.add_action(goal_x)
    ld.add_action(goal_y)
    ld.add_action(goal_z)

    ld.add_action(safety_checker)
    ld.add_action(moveit_control)

    return ld