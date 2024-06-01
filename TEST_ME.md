## simpleRosControl
### Setup
make new workspace to get rid of package dependancies
```bash
# Make catkin workspace
mkdir -p ~/cobotSim_ws/src
cd ~/cobotSim/src

# Clone git repo
git clone https://github.com/JerrysTurn/simpleRosControl.git

# Catkin workspace
cd ..
catkin_make
```

### spawn robot
```bash
roslaunch cobot_simulation spawn_robot.launch
```
joint value is not fixed in this point. So when sim time pass, manipulator fall down

### spawn robot with controller
```bash
roslaunch cobot_simulation spawn_robot_w_controller.launch
```
effort_controllers for hardware & JointPositionController for controller manager
1. Use rostopic message publisher to control joint
2. Use visualize plot to compare joint command & joint process value
3. Use dynamic configuration to fine-tuning pid value

![controller](images/cobot_w_controller.png)
### Add camera at the top of manipulator
Add gazebo plugin for depth camera & include dae, stl, png file for visualization
```
cobot_camera.xacro
└── kinetic_rgb.urdf.xacro
   ├── /meshes/sensors/camera/kinetic.dae
   ├── /meshes/sensors/camera/kinetic.stl
   ├── /meshes/sensors/camera/kinetic.png
   └── kinetic_rgb.gazebo.xacro s
```

```bash
roslaunch cobot_simulation spawn_robot_cube_camera.launch
```
![camera](images/cobot_w_camera.png)

### moveit configuration installation
```bash
sudo apt-get install ros-noetic-moveit ros-noetic-moveit-plugins ros-noetic-moveit-planners
roslaunch moveit_setup_assistant setup_assistant.launch
```
To make moveit controller work, joint_link.xacro file hardware_interface need to be changed from "EffortJointInterface" to "PositionJointInterface".

[Information about moveit has to be updated!!]

## etc
Check out original repository!! 
[repo1](https://github.com/LearnRoboticsWROS/cobot), [repo2](https://github.com/LearnRoboticsWROS/cobot_moveit_config_gazebo)