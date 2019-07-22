# MANIsimulation
The gazebo simulation of the MANI rover.

gazebo system requirements: http://wiki.ros.org/simulator_gazebo/SystemRequirements

package dependencies:
- ros_control: http://wiki.ros.org/ros_control#Install
- gazebo_ros_pkgs: http://gazebosim.org/tutorials?tut=ros_installing


# MANIsimulation
The gazebo simulation of the MANI rover. The simulated rover can be controlled via the respective ROS topics. It can also be controlled via a joint stick if used in conjunction with the [MANIros](https://github.com/PTScientists/MANIros) package.

This package requires familarity with ROS kinetic kame and Gazebo.

## Getting Started

Clone MANIsimulation into your catkin workspace (into catkin_ws/src). 

### Prerequisites

1. Install `ROS kinetic`: http://wiki.ros.org/kinetic/Installation

2. Install [`Gazebo`](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install):

``` 
curl -sSL http://get.gazebosim.org | sh 
```

3. Install [`ros control`](http://wiki.ros.org/ros_control#Install):

```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

4. Install [`gazebo_ros_pkgs`](http://gazebosim.org/tutorials?tut=ros_installing):
```  
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```

5. Install Catkin and set up a Catkin Workspace
If you are not familiar with catkin: http://wiki.ros.org/catkin/Tutorials

```
sudo apt-get install ros-kinetic-catkin
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

## Gazebo

## ROS integration

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

* Hat tip to anyone whose code was used
* Inspiration
* etc

