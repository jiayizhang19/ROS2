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

### - Run ROS2 Packages, see more commands in ROS2_Commands.md
``` bash
colcon build
source install/setup.bash
ros2 run pkg_name code_entry # code_entry points to the name in setup.py console_scripts
ros2 launch robot_bringup_folder bringup_name # control robots with one terminal instead of various ones, see examples in lesson_2
```

### - Folder Structures inside src
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

### RX-200 
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




