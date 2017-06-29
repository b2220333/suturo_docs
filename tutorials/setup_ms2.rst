ms2-setup
=========

Start the robot according to the tutorial.

Manipulation:
-------------
Branch: master

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

Without the visualizer:
	rosrun percepteros caterrosRun cateros.xml

	roslaunch percepteros cateros.launch 


run the Publisher:
	rosrun publisher publisher_node 

The visualizer flag is only needed if you want to have the visualizer running, if not you can just leave it out.

Knowledge
---------
Branch: misc/merge-ms3...

	roslaunch object_state object_state.launch

publish dummy perception:

rosservice call /json_prolog/simple_query "mode: 0
id: '100'
query: 'dummy_perception_with_close3(box)'" 
ok: True
message: ''


rosservice call /json_prolog/next_solution "id: '100'" 
status: 3
solution: {}




Pepper
------

	roslaunch dialogsystem dialog.launch

debug: check the launch file for right IP and stuffs like dat.




------------------------------------------
hasu@hawkin-suturo:~/suturo_ws/src/knowledge/object_state/prolog$ rosservice call /json_prolog/next_solution "id: '1'" 
status: 3
solution: {}
hasu@hawkin-suturo:~/suturo_ws/src/knowledge/object_state/prolog$ rosservice call /json_prolog/simple_query "mode: 0
id: '3'
query: 'dummy_perception_with_close3(box)'" 
ok: True
message: ''
hasu@hawkin-suturo:~/suturo_ws/src/knowledge/object_state/prolog$ rosservice call /json_prolog/next_solution "id: '3'" 
status: 3
solution: {}
hasu@hawkin-suturo:~/suturo_ws/src/knowledge/object_state/prolog$ 
