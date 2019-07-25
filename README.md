# MANIsimulation
The gazebo simulation of the MANI rover. The simulated rover can be controlled via the [respective ROS topics](https://github.com/PTScientists/MANIsimulation#manisimulation-and-ros). It can also be controlled via a joint stick if used in conjunction with the [MANIros](https://github.com/PTScientists/MANIros) package.

This package requires familarity with [ROS](http://www.ros.org/about-ros/) and [Gazebo](http://gazebosim.org/). Please also check whether your graphic card is [compatible with Gazebo](http://wiki.ros.org/simulator_gazebo/SystemRequirements).

#### Table of Contents
- [Getting Started](https://github.com/PTScientists/MANIsimulation#getting-started)
	- [Prerequisites](https://github.com/PTScientists/MANIsimulation#prerequisites)
	- [Installing](https://github.com/PTScientists/MANIsimulation#installing)
- [Spawning MANI in a (custom) world](https://github.com/PTScientists/MANIsimulation#spawning-mani-in-a-custom-world)
- [Gazebo Integration](https://github.com/PTScientists/MANIsimulation#gazebo-integration)
	- [Gazebo Overview](https://github.com/PTScientists/MANIsimulation#gazebo-overview)
	- [MANI in Gazebo](https://github.com/PTScientists/MANIsimulation#mani-in-gazebo)
- [ROS Integration (Robotic Operating System)](https://github.com/PTScientists/MANIsimulation#ros-integration-robotic-operating-system)
	- [ROS Overview](https://github.com/PTScientists/MANIsimulation#ROS-Overview)
	- [move MANI](https://github.com/PTScientists/MANIsimulation#move-mani)
	- [MANI joint data](https://github.com/PTScientists/MANIsimulation#mani-joint-data)
	- [MANI camera](https://github.com/PTScientists/MANIsimulation#mani-camera)
- [Built With](https://github.com/PTScientists/MANIsimulation#built-with)
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

_If you get an error, check whether your graphic card is [compatible with Gazebo](https://github.com/PTScientists/MANIsimulation/blob/master/README.md#gazebo-integration)._

![alt text](https://github.com/PTScientists/MANIsimulation/blob/master/mani_headshot_frontotherangle.png)

## Spawning MANI in a (custom) world

It is assumed that you have sourced your environment. It is also assumed that you have a catkin workspace called `catkin_ws`. 

#### Spawn MANI in an empty world
```$ roslaunch manisim gazebo.launch```

#### Spawn MANI in a custom world
1. Save a `.world` file in `~/catkin_ws/src/MANIsimulation/manisim_gazebo/worlds`. The manisim package comes with `empty.world` and `moon.world`. 
2. Launch MANIsimulation with the `world` command line argument. For example, if you want to launch MANI in `moon.world`, type:

```$ roslaunch manisim gazebo.launch world:=moon```

#### Set the position where MANI is spawned at
You can define the x-, y- and z-position MANI is spawned at when launching Gazebo. The default values are x=0, y=0 and z=0.2175. _(If z was 0, MANI would be placed slightly in the ground of empty.world. This is due to the position of MANI's own origin.)_

You can set each position coordinate individually. For example, to set x=5, type:

```$ roslaunch manisim gazebo.launch x:=5```

You can also set them all together. The order doesn't matter here:

```$ roslaunch manisim gazebo.launch y:=3.2 x:=5 world:=empty z:=1```

## Gazebo Integration
### Gazebo Overview
Gazebo is a 3D robotics simulator. It is open-source. You can find useful tutorials [here](http://gazebosim.org/tutorials). A good starting point for creating robot models in Gazebo is [here](http://gazebosim.org/tutorials?cat=build_robot).

Before using Gazebo, check out its system requirements [here](http://wiki.ros.org/simulator_gazebo/SystemRequirements). 

##### A WORD ABOUT GRAPHIC CARDS
There is a typical error called `BadDrawable` which is due to incompatible graphic cards/drivers. Often, this error occurs only sometimes during launch, and otherwise Gazebo launches up fine. If you get this error, please check whether your drivers are installed properly. You can also try to launch the MANIsimulation with roslaunch multiple times to see if it eventually launches up successfully. Feel free to also check out [this post](http://answers.gazebosim.org/question/3703/baddrawable-error-on-first-run-of-gzserver-after-install/) and [this post](https://answers.ros.org/question/27952/gazebo-shutdown-baddrawable/) regarding this issue.

### MANI in Gazebo
MANI consists of the following links:
- chassis
- camera_pan, camera_tilt, camera_kinect
- steering_angle_frontleft, steering_angle_rearleft, steering_angle_rearright, steering_angle_frontright
- wheel_frontleft, wheel_rearleft, wheel_rearright, wheel_frontright

`chassis` is the root link. The other links are all connected to the root/to each other via joints. The [ROS Integration section](https://github.com/PTScientists/MANIsimulation#ros-integration-robotic-operating-system) describes how to control these joints via ros topics.

MANI's links and joints are labelled based on their positions in respect to the robot itself. The positions are as follows:

![alt text](https://github.com/PTScientists/MANIsimulation/blob/master/mani_sidelabels.png)

## ROS Integration (Robotic Operating System)
### ROS Overview
[Wikipedia's Definition](https://en.wikipedia.org/wiki/Robot_Operating_System) _July 24th 2019, 15:57 CET_
>Robot Operating System (ROS or ros) is robotics middleware (i.e. collection of software frameworks for robot software development). Although ROS is not an operating system, it provides services designed for a heterogeneous computer cluster such as hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management.

To get started with ROS, you can find useful tutorials [here](http://wiki.ros.org/ROS/Tutorials).

### MANIsimulation and ROS
#### move MANI
You can move MANI in Gazebo by publishing to the following ROS topics. They expect a `std_msgs/Float64` message.
- Wheels (Velocity in m/s)
	- front left: `/manisim/drive_fl_vel/command`
	- rear left: `/manisim/drive_rl_vel/command`
	- rear right: `/manisim/drive_rr_vel/command`
	- front right: `/manisim/drive_fr_vel/command`
- Steering Angles (Orientation in rad)
	- front left: `/manisim/steer_fl_ort/command`
	- rear left: `/manisim/steer_rl_ort/command`
	- rear right: `/manisim/steer_rr_ort/command`
	- front right: `/manisim/steer_fr_ort/command`
- Camera Head (Orientation in rad)
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

## Authors

* **Nathalie Hager** - *Initial work* - [Git Profile](https://github.com/NathalieMH)

See also the list of [contributors](https://github.com/PTScientists/MANIsimulation/contributors) who participated in this project.

## License

At the time of writing this README, the license hasn't been decided upon yet. Please refer to the PTScientists Git Repo for more information.

## Acknowledgments

* [PTScientists](https://ptscientists.com)
