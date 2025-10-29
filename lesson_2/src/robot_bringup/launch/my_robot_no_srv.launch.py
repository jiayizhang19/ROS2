# Launch example where an incomplete system is loaded for debugging
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()        # Launch description object

    # Configure the nodes you want to add to the launch file
    loc_node = Node(                
        package='motor_control',    # Ensure you add the dependencies!
        executable='loc_status'     # The executable is the install name
    )
    syst_node = Node(
        package='motor_control',
        executable='rbt_status'
    )
    status_node = Node(
        package='motor_control',
        executable='status_manager'
    )
    # Add all the nodes to the launch description
    ld.add_action(loc_node)
    ld.add_action(syst_node)
    ld.add_action(status_node)
    # Return the launch description
    return ld