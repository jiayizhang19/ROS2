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

### Knowledge of Computer Vision
#### Principle of CV in Robotics
The goal is to **transform the object pose into the robot's coordinate system** so the robot can plan act. AprilTag is often used as a reference.
#### Setup 1: Fixed Camera + Tag on Arm
In this setup, the camera is the world reference, the AprilTag bridges the gap between camera and the robot.
1. The tag is rigidly attached to the arm, so its pose relative to the robot is known.
2. When the camera sees the tag, it knows the tag's pose relative to the camera. 
(The tag bridges the relationship between the camera and the robot system without extra calibration.)
3. TF Chain: camera_link --> tag_frame --> base_link
#### Setup 2: Moving Camera (mounted on EE) + Tag on Table
In this setup, the robot base is the world reference, the AprilTag between the camera and the object, but needs to know where the camera is in the robot's frame to complete the chain. This is why the calibration is needed.
1. When the camera sees the AprilTag, the tag's pose relative to the camera is known, which enables the transform of camera_link --> tag_frame.
2. The camera is mounted on the arm, the camera's pose relative to the robot is a constant, but we need to **compute the transform between the camera and end-effector**, where calibration is required, otherwise we do not know where the camera is in the robot frame.
3. TF Chain: base_link --> end_effector_link --> camera_link --> tag_frame.

### Implement Hand-Eye Calibration
1. Install Calibration Tools
Install easy_handeye, provides the GUI and algorithms for hand-eye calibration
```bash
sudo apt install ros-$ROS_DISTRO-easy-handeye ros-$ROS_DISTRO-apriltag-ros
```
2. Launch Robot and Camera
```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200
ros2 launch realsense2_camera rs_launch.py 
# if missing, run sudo apt install ros-$ROS_DISTRO-realsense2-camera
```
3. Launch AprilTag detection, apriltag_ros provides AprilTag detection and publishes camera_link --> tag_frame
```bash
ros2 launch apriltag_ros apriltag.launch.py camera_name:=camera
```
Verify that /tf publishes camera_link → tag_frame when the tag is visible.
4. Start Easy HandEye Calibration
```bash
ros2 run easy_handeye calibration_gui
```
Select Eye-in-Hand mode.
Set frames:
Robot base: base_link
Effector: end_effector_link
Camera: camera_link
Object: tag_frame
5. Collect Samples
Move the arm to 10-15 different poses where tag is visible. 
Click Add Sample each time.
When enough samples are collected, click Compute Calibration.
6. Save Calibration
Easy HandEye generates a YAML file with:
```yaml
camera_calibration:
  translation: [x, y, z]
  rotation: [qx, qy, qz, qw]
```
7. Apply Calibration
URDF: hardcode the transform in the robot description. If the camera is moved physically, must update the URDF and restart the robot description.
```xml
<joint name="camera_mount" type="fixed">
  <parent link="end_effector_link"/>
  <child link="camera_link"/>
  <origin xyz="X Y Z" rpy="Roll Pitch Yaw"/>
</joint>
```
OR 
Static Transform Publisher: provides the transform in the launch file. If the camera postion changes, still needs to update these values manually.
```py
<node pkg="tf2_ros" exec="static_transform_publisher"
      args="X Y Z Roll Pitch Yaw end_effector_link camera_link"/>
```