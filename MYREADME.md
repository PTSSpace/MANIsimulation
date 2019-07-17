to launch Mani in Gazebo:

1. $ cd ~/catkin_ws	 (i.e. your catkin workspace)
2. $ source /opt/ros/kinetic/setup.bash
3. $ source ~/catkin_ws/devel/setup.sh OR $ source ~/catkin_ws/devel/setup.bash
4. $ roslaunch manisim gazebo.launch

to control mani's wheels and steering angles:

-> 1st option: use rqt
1. $ rosrun rqt_gui rqt_gui
2. from the menu bar: Plugins -> Topics -> Message Publisher
3. select /mani/[controllername]/command topic of the joint you want to control
4. publish velocity / position / etc. to topic (Float64)

-> 2nd option: publish to ros topics via the command line / via your own program (see step 3. in 1st option rqt)
