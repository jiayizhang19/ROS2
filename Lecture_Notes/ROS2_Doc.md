## Documentation 
https://docs.ros.org/en/foxy/index.html -- oldest version, already EOL  
https://docs.ros.org/en/humble/index.html -- ubuntu 22.04

### Create ROS2 Packages
```bash
mkdir folder_name
cd folder_name
mkdir src
cd src
ros2 pkg create --build-type ament_python robot_pkg_name
# remove a folder
rm -rf folder_name
```

### Run ROS2 Packages (see more commands in ROS2_Commands.md)
``` bash
colcon build
source install/setup.bash
ros2 run pkg_name code_entry # code_entry points to the name in setup.py console_scripts
ros2 launch robot_bringup_folder bringup_name # control robots with one terminal instead of various ones, see examples in lesson_2
```

### Folder Structures (inside src)
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


### Publisher-Subscriber
- Communication Pattern: One-way, **asynchronous**
- Publisher sends messages of a specific type (e.g. std_msgs/Bool) to a topic, subscriber receives messages from that topic and process them.
- It uses **message types (e.g. std_msgs.msg/Bool)** as it only sends data one way.
- Workflow:
    - Publisher: Create Publisher (define message type & topic name) --> **Publish Data using self.pub.pubish(msg)**
        - Inside a timer callback if needs a periodic publishing, self.create_timer(1.0, self.pub_function) OR
        - Inside a subscriber callback or any event-driven function
    - Subscriber: Create Subscriber (define message type & topic name) --> Define Callback Function
        - Receives msg automatically
        - Perform actions based on msg
        - Publish something if needed
```python
#############################################################
###################### Subscriber node ######################
#############################################################
from std_msgs.msg import String
class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node')
        self.sub = self.create_subscription(String, '/topic_name', self.callback, 10)
        # Only one argument (msg) because the subscriber only needs the incoming message, and the callback gets msg automatically.
    def callback(self, msg): # it always has msg as an argument because ROS2 injects it
        self.get_logger().info(msg.data)
############################################################
###################### Publisher node ######################
############################################################
class pub_node(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.pub = self.create_publisher(String, '/topic_name', 10)
        self.timer = self.create_timer(1.0, self.pub_function)
    def pub_function(self):
        msg = String()
        msg.data = 'Hello'
        self.pub.publish(msg)
```

### Services (Server-Client)
Services are based on a **call-and-response** model, versus topics' publisher-subscriber model. While topics allow nodes to subscribe to data streams and get continual updates. Services only provide data when they are specifically called by a clinet.
- Communication Pattern: Request-response, **synchronous**
- Service provides a function that clinets can call, client sends a request and waits for a response.
- It uses **service types (std_srvs.srv/SetBool)** because it needs both a request and a response strucure.
- Workflow:
    - **Server**: Create a Service with service type and topic name --> Define a Callback (request, response)
        - Process logic based on request
        - Populates response fields
        - Return the response (mandatory for client to receive it)
    - **Client**: Create a Client with service type and topic name --> Wait for Service Availability using wait_for_service() --> **Set a timer to Send Request** --> Write a funtion to send request
        - initiate request using request = Service_Type.Request()
        - send request asynchronously uses call_async(). (The reason use call_async() here is using synchronous call() will block the client node until the server responses, while call_async() keeps the client node running while waiting for response.)
        - register a callback function with .add_done_callback() to handle the response when ready (without this step, everything still works but you'd need to manually check future result)
```python
from std_srvs.srv import SetBool
#########################################################
###################### Server Code ######################
#########################################################
# SetBool is a service type, which defines both request and response fields
self.creat_service(SetBool, '/motor_relay', self.motor_relay_service_server)
# The server receives a request and must return a response
def motor_relay_service_server(self, request, response):
    # Takes the request from the data field
    req = request.data
    # Create a string based on the request
    string = 'Activated motor' if req else 'Operation failed'
    # Log it
    self.get_logger().info(string)
    # And formulate the response to be returned
    response.success = True
    response.message = string
    return response
#########################################################
###################### Client Code ###################### 
#########################################################
self.cli = self.create_clinet(SetBool, 'motor_relay')
# ROS2 nodes often start at different times. It keeps checking every second until the service is available, otherwise the client may try to send a request before the server is ready, and so the request will fail
while not self.cli.wait_for_service(1.0):
    self.get_logger().info('Waiting for server...')
# Calls self.control_motor() every 3 seconds, without this, the client node will never send requests to server automatically
self.create_timer(3.0, self.control_motor)

# The client must initiate a request
def control_motor(self):
    # Create a request object manually
    request = SetBool.Request()
    request.data = True
    # Then send the request asynchronously using call_async(), it returns a Future object, which represents a result that will be available later
    # The Future object represents a placeholder for a result that will be available later.
    future = self.cli.call_async(request)
    # Register a function to be called when the Future is completed (i.e., when the service response arrives)
    future.add_done_callback(self.motor_control_future_callback)

def motor_control_future_callback(self, future):
    self.get_logger().info('Calling from the future')
    # Access the response
    response = future.result()
    self.get_logger().info(f'Service response: {response.message}')
```


### Declare Parameters in the Terminal
1. **Define** a parameter that can be set from the terminal
```Python
# In launch file, declare a launch argument named xs
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
start_x = DeclareLaunchArgument('xs', default='0.25', description='x coordinate of start postion')
```
2. **Retrieve** the value of that parameter
```python
# In launch file, use the value of the xs launch argument (either the default or the one passed from the terminal)
'xs': LaunchConfiguration ('xs'),
```
3. **Pass** the values to the node
```python 
# These parameters are passed to the node rx200_moveit_client, which can then access them using self.get_parameter('xs').value
moveit_control = Node(
    package='rx200_moveit_control',
    executable='rx200_moveit_client',
    name='rx200_moveit_control',
    parameters=[{
        # Pass the value of 'xs' (the second) as a parameter to the node 'xs' (the first)
        # The first xs must match the name used in self.get_parameter() and self.declare_parameter() -- used inside the node
        # The second xs must match the name used in DeclareLaunchArgument() -- used in the launch file
        'linear': LaunchConfiguration('linear'),
        'xs': LaunchConfiguration ('xs'),
        'ys': LaunchConfiguration ('ys'),
        'zs': LaunchConfiguration ('zs'),
        'xg': LaunchConfiguration ('xg'),
        'yg': LaunchConfiguration ('yg'),
        'zg': LaunchConfiguration ('zg')
    }]
)
```
4. **Read** values inside the node
```python
# In ROS2, every parameters MUST be declared before it can be accessed using get_parameter(). If xs was not declared, it will raise an error using get_parameter()
# The value 0.25 here is the fallback in case it's run standalone without a launch file, or the parameter is missing.
self.declare_parameter('xs', 0.25)
# Retrieve and use value passed from the launch file / terminal
self.x_start = self.get_parameter('xs').value
```


### Rotation
Quaternion is a mathematical way to represent rotations in 3D place. It's an alternative to Euler angles (roll,pitch, yaw) and rotation matrices.
Quaternion q has four components:
- x,y,z: The vector part (axis of rotation)
- w: The scalar part (cosine of half the rotation angle, cos(theta/2))
All the above four components are calculated on (roll, pitch, yaw) by certain formulas, so if wants robot to rotate, all these four components must be changed, not just w. 

```python 
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Quaternion,Point
class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')
        self.group_name_arm= 'interbotix_arm'
        self.group_name_gripper= 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
    def send_ee_pose(self, x, y, z, task=None):
            pose = PoseStamped()
            pose.header.frame_id = self.base_link # refenence to base_link
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            # pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # no orientation
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 1.57) # rotation angle in x, y, z axis separately, 1.57 = pai / 2
            pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
```




