### Commands to bringup RViz with a "fake" robot arm, update with "actual" for a real arm
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=fake
```

### Update start and target points
Define start and target(goal) points in src/rx200_moveit_control/rx200_moveit_control/config/poses.yaml


### Command to run the node
```bash
ros2 launch robot_bringup control_only.launch.py
```
