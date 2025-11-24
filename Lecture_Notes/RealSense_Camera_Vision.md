### Commands to run vision-based pick-and-place demo:
```bash
cd interbotix_ws/
# Activate / Deactivate the torque
ros2 service call /rx200/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'arm', enable: false}"
# Launch RViz and computer vision (replace rx200 with wx200 when running the demo)
ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=rx200 use_pointcloud_tuner_gui:=true use_armtag_tuner_gui:=true
# Run pick-and-place demo
python3 src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/demos/pick_place.py
```