# Drone Pose Control

ROS2 implementation of a SJTU drone movement controller trough human pose estimated using webcam. 

**Configuration**:

Compile the project:

```bash
cd ros2_ws/
colcon build
```

Launch pose estimation, movements interpreter and Gazebo simulator nodes:

```bash
ros2 launch launch_pkg  pose_control.launch.py
```

Wait for simulator launch, then, in another terminal, intialize drone flight mode:

```bash
ros2 run gesture_controller drone_initializer
```

Now is possible to control the drone using the folliwing gestures:
- Ascent: raise right hand
- Descent: raise left hand
- Hover: both hands along the sides
- Foward: bring the right hand next to the right shoulder
- Backward: bring the left hand next to the left shoulder
- Right rotation: extend the right arm parallel to the ground
- Left rotation: extend the left arm parallel to the ground
