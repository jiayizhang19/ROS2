## Documentation 
https://docs.ros.org/en/foxy/index.html -- oldest version, already EOL
https://docs.ros.org/en/humble/index.html -- ubuntu 22.04

### Create ROS2 Packages
```bash
mkdir folder_name
cd folder_name
mkdir src
cd src
ros2 pkg create --build-type ament-python robot_pkg_name
# remove a folder
rm -rf folder_name
```

### Run ROS2 Packages (see more commands in ROS2_Commands.md)
``` bash
colcon build
source install/setup.bash
ros2 run pkg_name code_entry # code_entry points to the name in setup.py console_scripts
ros2 launch robot_bringup_folder bringup_name # control robots with one terminal instead of various ones, see examples in lesson_2
```

### Folder Structures (inside src)
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


### Services
Services are based on a **call-and-response** model, versus topics' publisher-subscriber model. While topics allow nodes to subscribe to data streams and get continual updates. Services only provide data when they are specifically called by a clinet.

#### Server
#### Clinet


### Declare Parameters in the Terminal
1. **Define** a parameter that can be set from the terminal
```Python
# In launch file, declare a launch argument named xs
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
start_x = DeclareLaunchArgument('xs', default='0.25', description='x coordinate of start postion')
```
2. **Retrieve** the value of that parameter
```python
# In launch file, use the value of the xs launch argument (either the default or the one passed from the terminal)
'xs': LaunchConfiguration ('xs'),
```
3. **Pass** the values to the node
```python 
# These parameters are passed to the node rx200_moveit_client, which can then access them using self.get_parameter('xs').value
moveit_control = Node(
    package='rx200_moveit_control',
    executable='rx200_moveit_client',
    name='rx200_moveit_control',
    parameters=[{
        # Pass the value of 'xs' (the second) as a parameter to the node 'xs' (the first)
        # The first xs must match the name used in self.get_parameter() and self.declare_parameter() -- used inside the node
        # The second xs must match the name used in DeclareLaunchArgument() -- used in the launch file
        'linear': LaunchConfiguration('linear'),
        'xs': LaunchConfiguration ('xs'),
        'ys': LaunchConfiguration ('ys'),
        'zs': LaunchConfiguration ('zs'),
        'xg': LaunchConfiguration ('xg'),
        'yg': LaunchConfiguration ('yg'),
        'zg': LaunchConfiguration ('zg')
    }]
)
```
4. **Read** values inside the node
```python
# In ROS2, every parameters MUST be declared before it can be accessed using get_parameter(). If xs was not declared, it will raise an error using get_parameter()
# The value 0.25 here is the fallback in case it's run standalone without a launch file, or the parameter is missing.
self.declare_parameter('xs', 0.25)
# Retrieve and use value passed from the launch file / terminal
self.x_start = self.get_parameter('xs').value
```


### Rotation
Quaternion is a mathematical way to represent rotations in 3D place. It's an alternative to Euler angles (roll,pitch, yaw) and rotation matrices.
Quaternion q has four components:
- x,y,z: The vector part (axis of rotation)
- w: The scalar part (cosine of half the rotation angle, cos(theta/2))
All the above four components are calculated on (roll, pitch, yaw) by certain formulas, so if wants robot to rotate, all these four components must be changed, not just w. 

```python 
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Quaternion,Point
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




