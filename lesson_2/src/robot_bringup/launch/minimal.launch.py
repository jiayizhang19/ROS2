# Minimal launch file example -- here we are not launching anything
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    '''Ensure you have the '''
    ld = LaunchDescription()        

    # Create you node actions using the Node object
    # Add nodes using ld.add_action

    return ld