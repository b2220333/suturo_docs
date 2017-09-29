
Turtlebot
=========

The following will include how to setup the Turtlebot itself in order for it to be used with the PR2 as a ROSmaster, how to maintain the Tortugabot and also what the high-level interface includes. 


Tortugabot handling & startup
------------------------------
1. Turn on the Laptop on top of the Robot and log in. This might take a while to boot. Generally keep it plugged in whenever you are not driving the robot, so that it is always charged when you actually want to drive the robot around. 

2. Connect it to the correct wifi, which is the PR2WLAN24. 

	TROUBLESHOOTING: If you don't see the wifi icon, try to plug out and back in the USB cable, which connects the laser scanner to the laptop. If it doesn't reappear, reboot the laptop. If it is still gone, reboot again, but this time during the loading screen of ubuntu, when it says "configuring Network", plug out and in the USB connector again. After that, it should have wifi. 

3. We assume all the necessary packages are already set up (which would be tortugabot_brngup, amcl, move_base, navigate_map...) and that you have pulled the sut_tortugabot package. If you aren't sure, just go to a terminal and see if you can roscd into them.

4. Plug in the battery. 
	
	Note: Tortugabot1 has one battery while Tortugabot2 has two connected batteries, acting as one, since it has bigger motors. 

5. open a terminal on the robot or connect to it via ssh. Type:
	
	pr2master

Which is an alias to set the PR2 as master. Then open byobu.

6. execute::
	
	roslaunch sut_tortugabot t1_complete.launch

This will launch internally (in summary)::

	t1_minimal_bringup.launch 			
	t1_amcl_laser_without_map.launch 	
	t1_move_base_laser.launch 			

The first one brings up the motordriver (roboclaw), the laser (hokyo), joystic teleop, and a few other things. (a little chain of launch files.) The second is amcl aka. localization. Without the map server since the map is published by the PR2. The last brings up move_base so that the robot can drive around.

Done!

Turtlebot batteries
--------------------
The thing with the batteries....
Keep in mind how much you use the robot, since there is no way of checking how full the battery is. At least, not without disconnecting it from the robot. So if you drive around a lot, you should probably charge it after 1-2 hours. If the robot is mostly standing still, you might charge it after 3-4 hours. 
It is important that the battery never runs low completly, since it might explode when completly depleated. 
To charge the battery, connect it to the charger (ask Alexis or Gaya which one that is). Make sure that the settings are: 
LiPo, charge, 2 Amp, V auto. If these are selected, hold the green button and the charger will beep and start chareging. 
For the Turtlebot1 battery the values the charger shows during chareging should be 3 cells, and around 12 V (+ - 1 or 2 Volts).
For the Turtlebot2 battery, it's 6 cells, 24 V (+ - 1 or 2 Volts). If the values are off, fetch the supervisors.

Don't just unplug the batteries during the chareging process. If you need to interrupt it, hols the stop button. The charger will beep and let you know that is has stopped. 

When the chareging process is completed, the charger will beep a few times. Wait till it stops beeping. It will show that the batteries are Full. Press the stop button, and disconnect the batteries. 
If you don't use the battery anymore that day, please put it back into it's lipo saver bag. Just in case.



More Troubleshooting:
_____________________
If it cannot connect to Hokuyo or Roboclaw: try relaunching the launch file a few times, plugging in the battery in and out a few times. If that doesn't help, reboot, repeat untill it works. If not, call for help.

Note: In the package of sut_tortugabot, are many more launch files. Basically you can call amcl and move_base separately for debugging purposes, some files are being called by other files, so please, just keep them there.


The sut_tortugabot package
---------------------------

This package basically holds everything that needs to be executed localy on the Turtlebot itself. Which means, it holds a slightly adjusted roboclaw_node, the necessary parameters for move_base, the current map of the lab which we used and very many launch files, which are all prefixed.

What does prefixing mean?
__________________________
It means that if we want to use multiple Robots in the same network, with one common master, they will likely have several topics in common. And if we then go on and publish something on these topics, it will affect all robots at the same time. For example, if we publish something on the topic cmd_vel, all robots will start moving and it's likely that one of them will crash into something. We don't want that to happen, therefore we need to keep all the topics and nodes separated. Also, it makes identifying which robot runs what, much easier.  So now, instead of just having multiple topics called /cmd_vel, we have something like: /tortugabot1/cmd_vel and /tortugabot2/cmd_vel. The non prefixed topics are the ones of the PR2. But even though they are not prefixed, we can now tell them appart from the others. 

Changes to these launch files: 
_______________________________
These launch files are basically replicas of the basic tortugabot launch files, just that here, all the nodes and topics are prefixed and remapped so that everything the tortugabot does, has basically a prefix of tortugabot1 to it. This way, it won't clash. Also the Tortugabot has his own tf tree, which gets published to the the master on a regular basis, but slow frequency. 
Also we included some topic_tools nodes into some of these files, in order to remap and trottle down some of the publish rates of some of the turtle nodes. Otherwise it would flood the network with huge tf and laser scanner data, and loose localization in the process. 
Some of the values of amcl were also adjusted to the situation.

**some interesting amcl parameters**
**odom_alpha1-4:** these basically describe how much you trust your laser scanner data over odom. The higher this value, the more you prefer to trust your laser scanner and not odom. It might be viable to set this parameter high when your odom is unstable. Current value is 20.0. You can see which odom value describes what here: http://wiki.ros.org/amcl

**min_particles / max_particles:**
These particles describe the amount of assumed positions. (The little cloud of arrows which one sees in Rviz from the pose array.) Rule of thumb: if they are all over the place: it's not good. if they are all in one spot that it looks like there is just one or two arrows: that's bad as well. There should be a little cloud of them so to speak. In our case 100-200 were good values. You can also check the behaviour of the PR2 and it's pose array for reference. 

**use_map_topic** is a boolean. When set to true, amcl will listen to the /map topic instead of getting the map via service call. Set to false in our case.

**first_map_only** also a boolean. Uses the first map it receives and that's it.

All other parameters should be documented in amcl. 

Changes to roboclaw: 
____________________
The changes here are minor. Basically the topics are hardcoded and do not accept the parameters for frames like odom and base_footprint, so we had to prefix them manually here. Also, we commented out one diagnostic updates line, since it was making the robot lag terribly. 

Changes to the costmap parameters for move_base:
_________________________________________________
There is an own folder which holds all the parameters .yamls for move_base. Some of these values got adjusted as well. Generally, a good reference for calibrating is this http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide 
One important point to mention is the **sim_time** within the dwa_local_planner.yaml parameters. Setting this wrong can result in the robot spinning rather then moving towards it's goal (quote from that tutorial, actually, so check it out for further reference.) A good value is usually 1-2 seconds.