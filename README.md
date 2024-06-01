# Intelligent Robotics Project
Manipulator motion controller for obstacle avoidance in dynamic environments

## Getting Started

Project executed on Ubuntu 20.04 LTS with ROS1 Noetic version. If there is some need to install softwares, write how to install them in Prerequisites.

### Prerequisites

What things you need to install the software and how to install them

```
Give examples
```

### Installing

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

```bash
# TEST
```

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Referenc project & documentation

* [gazebo_panda](https://github.com/justagist/gazebo_panda) - Integrating franka_panda_description package with gazebo_ros_control 
* [franka_panda_description](https://github.com/justagist/franka_panda_description) - visual & collision meshes, urdf, xacro files with hardware_interface for gazebo is included. Dynamic parameter are estimated followed by [this paper](https://inria.hal.science/hal-02265293/document)
* [Robotics-toolbox](https://github.com/petercorke/robotics-toolbox-python) - NEO: A Novel Expeditious Optimisation Algorithm for Reactive Motion Control of Manipulators in IEEE Robotics and Automation Letters are implemented in our projects.

## Etc
franka_panda_description/config/panda_sim_controllers.yaml에서 hardware_interface와 controller_manager parameter를 joint별로 설정한다.


<!-- ## Intelligent Robotics project
### Setup
Environment: ROS1 noetic 
```bash
# TEST CODE
```

## Reference
gazebo panda 
https://github.com/justagist/gazebo_panda

franka panda description
https://github.com/justagist/franka_panda_description

## ETC
config panda_sim_controllers.yaml에서 hardware_interface와 controller_manager parameter를 joint별로 설정한다. -->
