# MANIsimulation
The gazebo simulation of the MANI rover. The simulated rover can be controlled via the respective ROS topics. It can also be controlled via a joint stick if used in conjunction with the [MANIros](https://github.com/PTScientists/MANIros) package.

This package requires familarity with [ROS](http://www.ros.org/about-ros/) and [Gazebo](http://gazebosim.org/). Please also check whether your graphic card is [compatible with Gazebo](https://github.com/PTScientists/MANIsimulation/blob/master/README.md#gazebo).

## Getting Started

The following sections will guide you through the installation of the MANIsimulation package and its dependencies.

### Prerequisites

1. Install `ROS kinetic`: http://wiki.ros.org/kinetic/Installation

The MANIsimulation package requires ROS **Kinetic Kame**. Desktop-Full install is recommended for this package.

2. If you didn't install Gazebo in step 1 (Gazebo7 is included in the ROS Kinetic Kame Desktop-Full install), install [`Gazebo`](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) now: 
``` 
$ curl -sSL http://get.gazebosim.org | sh 
```
You can test whether Gazebo is properly installed by typing ```$ gazebo```. An empty gazebo world should be launched.

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

It is assumed that you have installed catkin and sourced your environment. It is also assumed that you already have a catkin workspace called *catkin_ws*. 

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
_If you get an error, check whether your graphic card is [compatible with Gazebo](https://github.com/PTScientists/MANIsimulation/blob/master/README.md#gazebo)._

## Gazebo
gazebo system requirements: http://wiki.ros.org/simulator_gazebo/SystemRequirements
a word about graphic cards

## ROS Integration

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Nathalie Hager** - *Initial work* - [PTScientists](https://github.com/PTScientists)

See also the list of [contributors](https://github.com/PTScientists/MANIsimulation/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Many thanks to PTScientists.

