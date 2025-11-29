from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

####################################################################################################################################
#Example how to run the launch command
#ros2 launch rx200_moveit_control control_only.launch.py default_gr_state:=True xs:=0.25 ys:=0.0 zs:=0.02 xg:=0.40 yg:=-0.3 zg:=0.02
####################################################################################################################################

def generate_launch_description():
    ld = LaunchDescription()
    
    debug_state = DeclareLaunchArgument('debug', default_value='False', description='Verbose Mode')

    moveit_control = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_client',
    )
    #Vebose Mode
    ld.add_action(debug_state)

    ld.add_action(moveit_control)

    return ld
