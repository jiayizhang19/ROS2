I will add some useful terminal commands you should know about when working with ROS2. The best way to view this file is in VSCode (or other markdown editors/viewers)

### Useful Commands to use when debugging the node
```bash
ros2 node list # to see if your nodes are up and running
ros2 topic info /topic_name # to see how many subscribers and publishes are connected
ros2 topic echo /topic_name # to see the value of subscriber captured
```

### Issues to check if found the value does not get updated accordingly
If this value is initialized in the __init__(), you have to run the spin briefly to get the value updated.
Example in assignment_1 rx200_moveit_action_clinet.py.
```python
class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')
        self.is_safety_check_passed = False
    def call_safety_checks_for_points(self, x, y, z, label=''):
        # Check safety of the start point
        self.send_point_for_safety_check(x, y, z, label)
        # Important!!! Without rclpy.spin_once, the valule of is_safety_check_passed will not be updated, 
        # as the callback is not called while the node is initialized, so the value remains initialized value defined above.
        timeout = time.time() + 3.0  # 3 seconds from now
        while time.time() < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
```

### Check ROS_DOMAIN_ID on each computer
```bash
export ROS_DOMAIN_ID
gedit ~/.bashrc  # update ROS_DOMAIN_ID
```

### List all the nodes
```bash
ros2 node list
```

### List all the topics (you should see 2 default topics even when you are not running anything)
```bash
ros2 topic list
```

### List all the actions 
```bash
ros2 action list
```

### Check the type of a topic
```bash
ros2 topic type /chatter
```

### Get more information on a topic (type, and # of subs and pubs)
```bash
ros2 topic info /chatter
```

### See what is the "shape" of a message type
```bash
ros2 interface show geometry_msgs/msg/Twist
```

### Listen (subscribe) to a topic and print out everything on it
```bash
ros2 topic echo /chatter
```

### Publish to a topic (remove --once to publish at 1Hz consistently)
```bash
ros2 topic pub /chatter std_msgs/msg/String "{data: Cool_String}" --once
# If your string has spaces you will have to add single quotes!
ros2 topic pub /chatter std_msgs/msg/String "{data: 'Cool String123'}" --once
```

### Subscribe to a topic say /lala_count
```bash
ros2 topic echo /lala_count
```