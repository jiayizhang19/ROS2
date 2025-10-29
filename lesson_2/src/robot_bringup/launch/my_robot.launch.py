# A launch file that runs all the nodes in our robot's system
from launch import LaunchDescription    # Used for running the selected nodes
from launch_ros.actions import Node     # Used to parse the node to the L.D.


def generate_launch_description():
    ld = LaunchDescription()

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
    service_node = Node(
        package='motor_control',
        executable='motor_service'
    )
    # Add all the nodes to the launch description
    ld.add_action(loc_node)
    ld.add_action(syst_node)
    ld.add_action(status_node)
    ld.add_action(service_node)
    # Return the launch description
    return ld