## Documentation 
https://docs.ros.org/en/foxy/index.html

### - Create ROS2 Packages
```bash
mkdir folder_name
cd folder_name
mkdir src
cd src
ros2 pkg create --build-type ament-python robot_pkg_name
# remove a folder
rm -rf folder_name
```

### - Run ROS2 Packages (see more commands in ROS2_Commands.md)
``` bash
colcon build
source install/setup.bash
ros2 run pkg_name code_entry # code_entry points to the name in setup.py console_scripts
ros2 launch robot_bringup_folder bringup_name # control robots with one terminal instead of various ones, see examples in lesson_2
```

### - Folder Structures (inside src)
- robot_pkg_name
    - resource
    - code_directory 
        - contain all ros2 codes
    - test
    - **package.xml**
        - **add dependencies here**
    - setup.cfg
    - **setup.py:** 
        - **update entry_points >> console_scripts**
- robot_bringup
    - lanuch 
        - my_robot.launch.py # see examples in lesson_2

### RX-200 Robot Arm
- Install
```bash
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```
- Connect to the real arm
```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200
cd interbotix_ws/
source install/setup.bash
# Disable the motor so that we can move it with hands
ros2 service call /rx200/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"
```
- Using MoveIt in RViz
```bash
cd interbotix_ws/
source install/setup.bash
# For turning on the arm using MoveIt in RViz
# Note: By replacing 'actual' in the command, you can launch without moving the real arm. For Gazebo simulation replace with 'gz_classic' and for RViz only replace with 'fake'.
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=actual
```
- Check action list and interface after running the ros2 MoveIt code
```bash
ros2 action list # there should be a /move_action
ros2 interface 
```

### ROS2 Rotation
Quaternion is a mathematical way to represent rotations in 3D place. It's an alternative to Euler angles (roll,pitch, yaw) and rotation matrices.
Quaternion q has four components:
- x,y,z: The vector part (axis of rotation)
- w: The scalar part (cosine of half the rotation angle, cos(theta/2))
All the above four components are calculated on (roll, pitch, yaw) by certain formulas, so if wants robot to rotate, all these four components must be changed, not just w. 

```python 
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')
        self.group_name_arm= 'interbotix_arm'
        self.group_name_gripper= 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
    def send_ee_pose(self, x, y, z, task=None):
            pose = PoseStamped()
            pose.header.frame_id = self.base_link # refenence to base_link
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            # pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # no orientation
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 1.57) # rotation angle in x, y, z axis separately, 1.57 = pai / 2
            pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
```




