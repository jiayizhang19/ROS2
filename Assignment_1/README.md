### High Level Architecture
- Project has 2 nodes. 
    1.  The main Node (MoveItEEClient) does the work. 
    2.  The second Node (PointSafetyCheck) is a simple publisher and subscriber node to show two way communication with the main node. At a later stage more functions can be offload to it. Like user input validation.

- Two types of motion: 
    1. Free Moveit motion 
    2. Linear motion in small linear increments.

- Four zones are defined: 
    1. Robot base, where collection/drop of objects is not allowed
    2. Areas close to robot base in where EE pitch is vertical
    3. Areas at distance from base that permit EE horizontal pitch
    4. Areas beyond the robot arm reach in where collection/drop of objects is not allow.

- All areas and parameters are define via .yaml file (poses.yamal) for easy and dynamic modification.

- User input is provided through command prompt for simplification

### Ways to run the Project
- Using below commands to run the project/nodes (launch file: control_only.launch)
```
source install/setup.sh
ros2 launch robot_bringup control_only.launch.py xs:=0.25 ys:=0.15 xg:=0.2 yg:=-0.3
```
Other Examples
```
ros2 launch robot_bringup control_only.launch.py xs:=0.25 ys:=0.15 xg:=0.2 yg:=-0.3 linear:=True
```
```
ros2 launch robot_bringup control_only.launch.py xs:=0.25 ys:=0.15 zs:=0.05 xg:=0.2 yg:=-0.3 zg:=0.05 linear:=False
```
### Parameters in use through command prompt
- start point (collection)
- target point (drop off)
- linear motion or free MoveIt mode
- All parameters have default values. Default movement mode is free MoveIt mode (linear:=False), z coordinate can be specified but is defaulted to 5cm(z:=0.05)

### Configuration parameters used are provided through .yaml file

Go to the below path to define all the parameters in one go. But it requires a compile for changes to take effect. Reason why user input was excluded.

Parameters included in .yaml:
- safe pose, smilar to Rviz sleep possition
- zones coordinates define as radiuses and xy coordinates (base, circutry box behind base)
- gripper status: open or close
- verbose mode for debug
- EE pitch angles

```
/src/rx200_moveit_control/rx200_moveit_control/config/poses.yaml
```

### Motion Strategy Explaination
1. move to the safe pose -->   
2. move above the start pose --> move down to the start pose --> move above the start pose    
3. move above goal pose --> move to the goal pose --> move up above goal pose -->   
4. back to the safe pose

#### Limitations and challenges
- Syncronous and Asyncronous nature of Actions and Topics still pose an understanding challenge
- Singularities of the arm are not well understood/controlled. Mainly driven by distant point or long travel distances.
- User input validation should be offload to a second node, as well as parameters loaded from .yaml.
- Location of .yaml file prevented the team from using it for user input. We have not solved the problem of hardcoded paths. We plan to continue this line of inquiry 
- Obstacle definition to prevent collitions needs to be incorportated to a future project. Some postion/movement cause the arm to hit the resting fork at the back of the base. Specially in linear mode movement.
- Ongoing goal is allowing user coordinate input through GUI.




