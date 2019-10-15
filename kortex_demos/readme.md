## Dependencies

### Dependencies (system)

```sudo apt install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev```

### Dependencies (ROS Kinetic)

```sudo apt-get install ros-kinetic-rgbd-launch```

### Dependencies (ROS Melodic)

```sudo apt-get install ros-melodic-rgbd-launch```

### Dependencies (pip2)

```pip2 install imutils```

## Description of the demo

### How to run the demo:

With the default IP address (192.168.1.10):

```roslaunch kortex_demos pickup_cube.launch```

With a non-default IP address (example with 10.0.101.184):

```roslaunch kortex_demos pickup_cube.launch ip_address:=10.0.101.184```

### How the demo works

Basically, the demo searches for a colored cube, then approaches it, then grabs it, then drops it. It loops this whole process.

Upon launching the launch file, the arm driver will start, as well as the vision driver and two image-processing nodes.
The main demo node (picukup_cube.py) will also start. 
The robot will then:
1. Reach the initial position and open the gripper.
2. Rotate the wrist one way, then the other way, then rotate back to the best orientation for the gripping to take place.
3. Approach the cube. This is the part where you can take the cube and move it away from the robot. If the robot still sees the cube in its FOV it will follow it. If the robot does not see the cube, it will go back towards its initial position until it sees the cube again.
4. When it has approached the cube enough, it will make its final approach. This approach is hardcoded and does not use visual servoing. 
5. It will then grab the cube (close the gripper).
6. The robot will go back to a transition position, where it will open the gripper to drop the cube. 
7. The steps 1-6 are repeated until the program is killed. 

### Parametrizing the demo

- We use small cubes of 56mm by 56mm by 56mm. If you use different cubes, you will have to change the ```self.gripper_closed``` variable in ```pickup_cube.py``` to grasp smaller or bigger cubes without squeezing them too much with the gripper. The value is between 0.0 (fully opened) and 1.0 (fully closed).
- We use a green cube, and the recognition is color-based. You can change the ```self.color_lower``` and ```self.color_upper``` variables (as RGB elements) in ```cube_xy_processing.py``` to respectively give the darker and lighter tones of your cube. This cannot be a fixed value, because the lighting conditions will vary the perception of the cube's color. 
- If you want to drop the cube from a different position, you can change the ```self.pose_transition``` variable (it's a cartesian pose).
