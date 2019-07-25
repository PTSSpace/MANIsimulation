# MANIsimulation
The gazebo simulation of the MANI rover. The simulated rover can be controlled via the [respective ROS topics](https://github.com/PTScientists/MANIsimulation#manisimulation-and-ros). It can also be controlled via a joint stick if used in conjunction with the [MANIros](https://github.com/PTScientists/MANIros) package.

This package requires familarity with [ROS](http://www.ros.org/about-ros/) and [Gazebo](gazebosim.org/). Please also check whether your graphic card is [compatible with Gazebo](https://github.com/PTScientists/MANIsimulation/blob/master/README.md#gazebo).

#### Table of Contents
- [Getting Started](https://github.com/PTScientists/MANIsimulation#getting-started)
	- C [Prerequisites](https://github.com/PTScientists/MANIsimulation#prerequisites)
	- [Installing](https://github.com/PTScientists/MANIsimulation#installing)
- [Spawning MANI in a custom gazebo world](https://github.com/PTScientists/MANIsimulation#spawning-mani-in-a-custom-gazebo-world)
- [Gazebo Integration](https://github.com/PTScientists/MANIsimulation#gazebo-integration)
- [ROS Integration (Robotic Operating System)](https://github.com/PTScientists/MANIsimulation#ros-integration-robotic-operating-system)
- [Built With](https://github.com/PTScientists/MANIsimulation#built-with)
- Contributing
- Versioning
- [Authors, License, Acknowledgements](https://github.com/PTScientists/MANIsimulation#authors)

## Getting Started

The following sections will guide you through the installation of the MANIsimulation package and its dependencies.

### Prerequisites

1. Install `ROS kinetic`: http://wiki.ros.org/kinetic/Installation

The MANIsimulation package requires ROS **Kinetic Kame**. Desktop-Full install is recommended for this package.

2. If you didn't install Gazebo in step 1 (Gazebo7 is included in the ROS Kinetic Kame Desktop-Full install), install [`Gazebo`](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) now. You can test whether Gazebo is properly installed by typing ```$ gazebo```. An empty gazebo world should be launched.
``` 
$ curl -sSL http://get.gazebosim.org | sh 
```

3. Install [`ros_control`](http://wiki.ros.org/ros_control#Install): 
```
$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

4. Install [`gazebo_ros_pkgs`](http://gazebosim.org/tutorials?tut=ros_installing):
```  
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```

5. Install Catkin and set up a Catkin Workspace:
	
	- install catkin 
	```
	$ sudo apt-get install ros-kinetic-catkin
	```
	- create and build a catkin workspace
	```
	$ source /opt/ros/kinetic/setup.bash
	$ mkdir -p ~/catkin_ws/src
	$ cd ~/catkin_ws/
	$ catkin_make
	```
	- source your new setup.*sh file:
	```
	$ source devel/setup.bash
	```

If you are not **_very_** familiar with catkin and/or would not be able to explain the steps taken above: http://wiki.ros.org/catkin/Tutorials

### Installing

It is assumed that you have installed catkin and sourced your environment. It is also assumed that you already have a catkin workspace called `catkin_ws`. 

1. Clone MANIsimulation into your catkin workspace.

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/PTScientists/MANIsimulation.git
```

2. Build your catkin workspace.
```
$ cd ~/catkin_ws
$ catkin_make
```

3. Launch the simulation.
```
$ roslaunch manisim gazebo.launch
```
You should see a MANI rover in an empty gazebo world. Have fun! 

![alt text](https://github.com/PTScientists/MANIsimulation/blob/master/mani_headshot_frontotherangle.png)

_If you get an error, check whether your graphic card is [compatible with Gazebo](https://github.com/PTScientists/MANIsimulation/blob/master/README.md#gazebo)._

## Spawning MANI in a custom Gazebo world

lorem ipsum dolor sit amet

## Gazebo Integration
### Gazebo Overview
Gazebo is a 3D robotics simulator. It is open-source.

You can find useful tutorials [here](http://gazebosim.org/tutorials). A good starting point for creating robot models in Gazebo is [here](http://gazebosim.org/tutorials?cat=build_robot).

gazebo system requirements: http://wiki.ros.org/simulator_gazebo/SystemRequirements
a word about graphic cards

### MANI in Gazebo
- insert image of mani with front/rear/left/right labels here -
- insert image of mani with joints here -

## ROS Integration (Robotic Operating System)
### ROS Overview
[Wikipedia's Definition](https://en.wikipedia.org/wiki/Robot_Operating_System) _June 24th 2019, 15:57 CET_
>Robot Operating System (ROS or ros) is robotics middleware (i.e. collection of software frameworks for robot software development). Although ROS is not an operating system, it provides services designed for a heterogeneous computer cluster such as hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management.

To get started with ROS, you can find useful tutorials [here](http://wiki.ros.org/ROS/Tutorials).

### MANIsimulation and ROS
#### move MANI
You can move MANI in Gazebo by publishing to the following ROS topics:
- Wheels
	- front left: `/manisim/drive_fl_vel/command`
	- rear left: `/manisim/drive_rl_vel/command`
	- rear right: `/manisim/drive_rr_vel/command`
	- front right: `/manisim/drive_fr_vel/command`
- Steering Angles
	- front left: `/manisim/steer_fl_ort/command`
	- rear left: `/manisim/steer_rl_ort/command`
	- rear right: `/manisim/steer_rr_ort/command`
	- front right: `/manisim/steer_fr_ort/command`
- Camera Head
	- camera pan: `/manisim/camera_pan_ort/command`
	- camera tilt: `/manisim/camera_tilt_ort/command`

#### MANI joint data
If you want to know the current position, velocity or effort of any of MANI's joints, subscribe to the `/manisim/joint_states` topic. For information on the joint states ROS message, look [here](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html). You can read up on `joint_state_publisher` [here](http://wiki.ros.org/joint_state_publisher).

#### MANI camera
The currently implemented depth camera for the MANI gazebo model is based on [the kinect camera setup described in this tutorial](http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros). This tutorial also explains how to visualize the generated point cloud in Rviz.

MANI's camera publishes to the following topics:
- /camera/depth/camera_info
- /camera/depth/image_raw
- /camera/depth/image_raw/compressed
- /camera/depth/image_raw/compressed/parameter_descriptions
- /camera/depth/image_raw/compressed/parameter_updates
- /camera/depth/image_raw/compressedDepth
- /camera/depth/image_raw/compressedDepth/parameter_descriptions
- /camera/depth/image_raw/compressedDepth/parameter_updates
- /camera/depth/image_raw/theora
- /camera/depth/image_raw/theora/parameter_descriptions
- /camera/depth/image_raw/theora/parameter_updates
- /camera/depth/points
- /camera_ir/depth/camera_info
- /camera_ir/parameter_descriptions
- /camera_ir/parameter_updates

## Built With

* [Gazebo](http://gazebosim.org) - Gazebo 3D Robotics Simulator
* [ROS](https://www.ros.org) - Robotic Operating System

## Contributing

??? Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

??? We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Nathalie Hager** - *Initial work* - [Git](https://github.com/NathalieMH)

See also the list of [contributors](https://github.com/PTScientists/MANIsimulation/contributors) who participated in this project.

## License

This project is licensed under the ??? License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Many thanks to [PTScientists](https://ptscientists.com).
