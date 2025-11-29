### 

#### Unit test of rx200_moveit_control
Command to run stack_manager:
```bash
ros2 launch robot_bringup rx200_control_only.launch.py xg:=0.40 yg:=-0.3 zg:=0.02 color_seq:='yellow, red, blue'
```

Command to publish coordinates to /object_detection
```bash
ros2 topic pub /object_detection std_msgs/msg/String "{data: 'red:5,20,3; blue: 5,0.2,0.3; yellow:5,10,10'}"  --once
```

Command to echo 
```bash
ros2 topic echo /object_detection
ros2 topic echo /stack_sequence
```