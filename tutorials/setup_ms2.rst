ms2-setup
=========

Start the robot according to the tutorial.

Manipulation:
-------------
Branch: feature/mergeMS2

On the Robot: (Make sure that the Robot's motors are turned of for this step!)
roslaunch giskard_examples pr2.launch sim:=false

Local:
roslaunch suturo_action_server pr2_action_server.launch 


Planning Seite dazu:
(start-ros-node "my_node")
(pr2-do::setup-move-robot-client)

Perception
----------
Branch: feature/knife-rgb  for Knife detection
(Branch: misc/milestone-test  for Cake/Box detection)

Requires: pcl 1.8 to run. Make also sure your RoboSherlock is up to date. 
( `pcl installation Tutorial: <http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php>`_. )

roslaunch robosherlock rs.launch

rosrun percepteros caterrosRun -visualizer cateros.xml


Knowledge
---------
Branch: ObjectDetection

roslaunch object_state object_state.launch

Pepper
------
