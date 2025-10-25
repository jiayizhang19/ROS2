I will add some useful terminal commands you should know about when working with ROS2. The best way to view this file is in VSCode (or other markdown editors/viewers)

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