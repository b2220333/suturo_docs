ms2-setup
=========

Start the robot according to the tutorial.

Manipulation:
-------------
Branch: feature/mergeMS2

On the Robot:
roslaunch giskard_examples pr2.launch sim:=false

Local:
roslaunch suturo_action_server pr2_action_server.launch 


Planning Seite dazu:
(start-ros-node "my_node")
(pr2-do::setup-move-robot-client)

Perception
----------
Branch: feature/knife-rgb
Requires: pcl 1.8 to run. Make also sure your RoboSherlock is up to date. 

roslaunch robosherlock rs.launch


Knowledge
---------
Branch: master

roslaunch object_state object_state.launch

Pepper
------