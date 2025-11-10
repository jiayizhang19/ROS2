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





