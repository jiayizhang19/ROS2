import os
import subprocess

workspace_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# print(workspace_path)


commands = """
colcon build && \
source install/setup.bash && echo "Environment sourced!" && \
# ros2 run rx200_moveit_control rx200_moveit_client
ros2 launch robot_bringup control_only.launch.py xs:=0.25 ys:=0.15 xg:=0.2 yg:=-0.3
"""

# Source the workspace and run the rx200 client
subprocess.run(
    ["bash", "-c", commands],
    cwd=workspace_path,
    check=True
)
