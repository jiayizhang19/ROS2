### Commands to bringup RViz with a "fake" robot arm, update with "actual" for a real arm
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=fake
```

### Parameters in use
Go to the below path to define all the parameters in one go. Parameters included are:
- safe pose
- movement strategy (prefer freestyle movement or staright-line movement)
- gripper status
- start point
- target/goal point
```
/src/rx200_moveit_control/rx200_moveit_control/config/poses.yaml
```


### Ways to run the node
- Run the runner file directly in below path
```
/src/rx200_moveit_control/run_rx200.py
```
- Using the below commands to run the nodes
```bash
colcon build
source install/setup.bash
ros2 launch robot_bringup control_only.launch.py
```

### System Design
#### Node 1: point_safetry_checker
- Checks start and goal points to see if they are reachable. 
- Reachable area are defined between the outest area the arm can reach and the least area without hitting the base.
#### Node 2: rx200_moveit_control 
- Contains main motion planning code, only if passing the point_safety_check should the path be planned and exectued.
                  

### Motion Strategy Explaination
1. move to the safe pose -->   
2. move above the start pose --> move down to the start pose --> move above the start pose  
3. back to the safe pose -->   
4. move above goal pose --> move to the goal pose --> move up above goal pose -->   
5. back to the safe pose
#### The basic and safe free movement

#### The advanced straight line movement -- need further development


### Useful Points used in test -- error points
(0.08,0.5,0.05) 
(0.51, 0, 0.05)
(-0.2, 0.09, 0.05)
