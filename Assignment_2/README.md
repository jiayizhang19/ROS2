### Code Structure
```
~/assignment_2/  
    ├── src/  
        ├── robot_bringup/                          # Launch Files
            ├── launch/  
                ├── rx200_control_only.launch.py    # Launch Stack_Manager & Moveit_Control Node
        ├── rx200_realsense_vision/                 # Robot Vision Design
        ├── rx200_moveit_control/                   # Robot Movement Design  
            ├── config/                             # .yaml configuration for static parameters
            ├── stack_manager.py                    # get cube info from camera & plan stack sequence
            ├── rx200_moveit_action_client.py       # pick cubes by sequence and stack to the target point
```
### High Level Node Communication Design
![High Level Node Communication Design](/src/HLD.png)



```
#===============================================================================#
#==================== Used for Jiayi's Intergrated Test Only ===================#
#===============================================================================#
```
Command to run stack_manager:
```bash
ros2 launch robot_bringup rx200_control_only.launch.py xg:=0.0 yg:=0.4 zg:=0.0 color_seq:='yellow, red, blue'
```

Command to publish coordinates to /object_detection
```bash
ros2 topic pub /object_detection std_msgs/msg/String "{data: 'red:0.1,0.20,0; blue:0.2,-0.2,0; yellow:0.10,-0.2,0'}"  --once
```

Command to echo on topics
```bash
ros2 topic echo /object_detection
ros2 topic echo /stack_sequence
```