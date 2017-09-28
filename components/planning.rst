.. note:: 
      Zur Dokumentation: Die Dokumentation soll so geschrieben sein, dass sie später anderen eine gute Übersicht über unser System gibt und dessen Nutzung vereinfacht. Wir sollen alle Komponenten unseres Systems (alle Features, die es so gibt oder Teile der Architektur, z.B. welche Dateien was enthalten, welche Regler es gibt...) beschreiben. Dabei helfen auch Grafiken, wie die Komponenten untereinander verbunden sind. Die Beschreibung soll eher auf "Highlevel"-Ebene sein, d.h. was die Features ungefähr machen und wo man sie findet, eine API ist nicht notwendig. Jeder sollte die von ihm entwickelten Features nach Möglichkeit selbst beschreiben!
      Memo: Die Notiz am Ende bitte wieder entfernen =)

Planning
========

.. note::
	Describing what Planning is all about and what nice and cool features we have.

Installation
------------

.. note::
	Simple. Just git clone. And rosinstall :D (in theory)

System Overview
---------------

The planning system of the CaterROS project controls all the other systems of the project and let's them work together. It uses the planning framework CRAM to control the PR2 and TurtleBot via flexible plans. The system communicates with the Pepper robot's systems to gather knowledge about the dialog with guests.

The main focus of the system is of course the execution of plans. There are two ways supported by the system: Direct and Guest-centered Execution. Direct means a user of the system executes a plan by calling the appropiate function and the system halts after the completion. The guest-centered on the other hand can run permanently until explicitly stoppped. It keeps track of the current state of the PR2 and decides what to do based on the guest's orders. This enables the system to dynamically react to changes in the guest's orders. Additionally to the predefined plans, the system also can generate plans based on the orders by guests.

Architecture
____________

Let's have a look at our architecture. The following picture depicts our modules and how they use each other.
	
.. note::
	Here should be a picture of the architecture.
Modules
_______

**planning_common**

The common module of our planning system. It holds functionality which is or was considered to be universally useful for our system. This ranges from simple utility functions and object definitions to the interface for calling prolog function of the knowledge base and robot-agnostic service calls and such. In here you can also find everything that didn't belong in the other packages but didn't warrant it's own package.


**Robot-specific command pools**

The robot-specific modules hold functionality especially necessary for that robot.

The pr2_command_pool package mostly holds calls to the action_move_robot function from planning_common to move the PR2. Some utility for these calls can also be found here.

The *turtle_command_pool* package holds everything which is needed to manipulate the turtle from the high-level, which means cram action an location designator definitions, the process module of the turtle and the action client.

	
**planning_communication**

The planning_communication module contains a RPC server implementation to enable the planning system to communicate with Pepper. It also contains a RPC client for the system to call functions of Pepper's RPC server.
Additionally the parser for Pepper's JSON-data-structures can be found here.


**plan_generator**

This module consists of an Python service to call the plan generator and the Lisp interface to generate problems for the generator and a parser for the generated plans.

	
**plan_execution**

The top-level module of our system. If you want to execute our plans, you have to load the system in this package. It contains all of our plans and the utilities to execute them. This includes our CRAM process modules and designator referencing.

The loop-function for the guest-centered plan execution can also be found here.


**sut_mockups**

There are mockups for all major nodes of the other systems. Those can be found in this package.


**planning_launch**

This package only holds two launch files for launching our mockups.


Plan Architecure
________________

As mentioned above we use the CRAM framework in our system. More specifically we use process modules and action designators to make our plans dynamic. For a more detailed look at CRAM itself check the `website <http://cram-system.org/>`_

In the planning_execution package are the files ``toplevel``, ``process-modules``, ``selecting-process-modules`` and ``action-designators``. Those contain everything that uses CRAM in our code. In ``toplevel`` we have the ``execute`` function which takes a string and executes the corresponding task. We define a task as a set of actions, or rather action designators. Each of the action designators gets referenced by using prolog to query the knowledge base. After this they get executed by the right process module. This happens automatically, as we have defined rules in ``selecting-process-modules`` to match process modules to designators. The process modules then directly call the plans in ``plans.lisp`` with the referenced information in the designator. The plans can also call sub-plans, but they don't use any more designators. All the information a single plan needs is obtained when an action designator is first referenced.

For the guest-centered execution we have the ``manager``. The name is a bit misleading, as this module doesn't actually manage the plan state or anything. It provides a function ``start-caterros`` which starts a loop that can be started anytime and then waits for guests to arrive. This is where the communication module comes into play. Through the RPC communication with Pepper the knowledge base gets updated parallel to the running loop. The loop can query the knowledge base through prolog.

Communication
-------------

There are three robots whose actions and knowledge are to combine. The planning_communication package prvides the communication between the Peppers dialogsystem and the ROS network, where the PR2 and Tortugabots are monitored. To include Pepper we implemented an RPC server on the Planning side, whose functions can be called from everywhere within the network, while concentrating on Peppers information. On the other hand the Planning side can feed Pepper with data and notify her about important changes in the world. To enhance the monitoring aspect of the Planning system, we save and update connection credentials of every system communicating with Plannings server.

**Setup Pepper Communication**

A setup file launches the RPC server and registers Peppers IP and Post to the list of available clients. Also it sends the IP and Port of the current machine to Pepper. This seput is called by the plan_execution init function, but if we want to look deeper into the planning_communication package, let's make the setup by ourselves. The final call, updating Peppers information about this machine, would fail anyway, if Peppers server isn't currently running.

**RPC-Server**

To initialize the RPC server, first load the planning_communication system in your REPL:

.. code:: lisp
	
	, r-l-s RET
	planning_communication RET
	RET
	,!p pcomm RET

Now that we work in the pcomm package, run the init-function of the server:

.. code:: lisp
	
	(init-rpc-server)
	
This function will simply start up a new ROSnode in the REPL and register all the functions provided by our RPC interface. The core functions used by Pepper are updateObserverClient, asserDialogElement, getGuestInfo and getAllGuestInfo.

**updateObserverClient** takes the ID of the robot (0 for Pepper), its ip as a string and its port as a number.
**assertDialogElement** takes a JSON string, that will be translated and forwarded to te knowledgebase. The whole variety of JSON queries is explained later. An example JSON string to order two pieces of cake looks like this:

.. code::

	{
	guestId:1,
	query: 	{
		type:setCake,
	  	amount:2,
	  	guestName:Arthur
	    	}
	}

The function will always answer the request with a JSON as well, telling if the request was processed successfully. This is the answer to the order sent previously:

.. code::

	{
	guestId:1,
	return: {
		type:setCake,
	  	success:1,
	  	tableId:table1
	    	}
	}

Only upon the request of a new order (type: setCake) the response contains the tableId of the guest, every other response lacks this information.

**getGuestInfo** needs a guest-id and returns all information about the order identified by this specific guest-id. A common response for the guest-id 1, considering we transmitted the order above, looks like this:

.. code::

	{
	guestId : 1,
	return: {	
		type: getGuestInfo,
		name: Arthur,
		location: table1, 
		total: 2,
		delivered: 0
		}
	}

**getAllGuestInfos** returns a list, containing all orders in the same format as a request for a specific guest (see **getGuestInfo**). It is called with any arbitrary parameter (there is a conflict when calling RPC function from Python to LISP, when the LISP function has no parameters).  

**RPC-Client**

The core functionality of the RPC client is to send RPC to Pepper. Mainly we use update-connection-credentials and fire-rpc-to-client. To make those calls more developer/user friendly, we have a list of clients, that use the Planning RPC server. We can take those connection credentials to fire a call to clients, using only their keynames.

**update-connection-credentials** will send the IP and port of the current machine (where the Planning server is running) to a remote client identified by its keyname, or to a yet unknown client using its IP and Port. The client must have an *updateObserverClient* function implemented on their side. After this call, the remote client will have information about our server. Here is an example usage:

.. code:: lisp
	
	(update-connection-credentials :client :pepper)

**fire-rpc-to-client** calls a function at a remote client. It uses the clients keyname, the function name and arguments needed in the function:

.. code:: lisp
	
	(fire-rpc-to-client :pepper "notify")

Plans/Actions
-------------

.. note::
	What actions do we have?
	
Executing Plans
---------------

There are two ways to execute the plans. Either by calling the ``execute`` function directly or by having guests in the knowledge base and let the system decide what to do on it's own.

**Setup**

To call the plans you need to load the ``plan-execution-system`` in the ``plan_execution_system``. So open up the roslisp REPL by opening a terminal and typing::

	roslisp_repl

In the REPL type::
	
	CL-USER> (ros-load:load-system "plan_execution_system" :plan-execution-system)
	
And go into the package::

	CL-USER> (in-package :pexecution)


**Direct**

Now you just have to call::

	PEXECUTION> (execute "demo")
	
To start the demo task. The task gets evaluated to designators and those get referenced to real plans. In ``toplevel.lisp`` is a function ``task->designators`` in which all the tasks and theirs corresponding designators are defined. The most important ones are the "steps", which can be executed in order to execute the whole scenario of the CaterROS project. The ``prep``, ``cut`` and ``deliver`` ones are also important as they are the ones called by the guest-centered method, but htey can also be executed directly.

**Guest-centered**

Now you call::

	PEXECUTION> (start-caterros)

This starts the guest-centered plan execution loop (or GCPEL, as I certainly will never call it). As long as there is no guest present in the knowledge base the loop prints a message that it's waiting for a guest. When a guest arrives and makes an order, the loop will start executing the plans. First it will execute the ``prep`` task, to grasp the tools. Then it will ``cut`` as often as the guest ordered pieces of cake. And lastly it will ``deliver`` the plate with the cake onto the TurtleBot, which will then bring it to the table.

If you want to test this without using Pepper`s Dialog system, you can call the ``test-guest`` function. It will generate a dummy guest in the knowledge base.

Plan Generation
---------------

The plan_generator module provides access to the classical planning system Fast Downward from http://www.fast-downward.org/ using a ROS service in python. It can be used to generate a plan for a given task within a given domain. In the case of the CaterROS café, it can be used to find a plan for the task of serving a given amount of pieces of cake in the CaterROS domain. Nevertheless, the underlying service can also be used for any other task and corresponding domain.

To use the plan generator for CaterROS, you have to: 
1. Follow the installation instructions at: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

2. Run the server for the python service 

      .. code:: bash

            rosrun plan_generator generate_plan.py

3. Start the demo as explanined at: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX



The Fast Downward planning system needs two inputs: a domain definition and a task definition written in the Planning Domain Definition Language (PDDL). You can find a good introduction on PDDL at: http://www.cs.toronto.edu/~sheila/2542/s14/A1/introtopddl2.pdf. 


provides a service that can be used to generate a plan for a given task within a given domain dynamically. The resulting plan is contained in a json string that can easily be transformed into a list of CRAM's action designators. 

In our implementation, the service is called within the plan_execution module. 

Fast Downward is based on the Planning Domain Definition Language (PDDL). The algorithm needs two files as input: a domain file and a task file. The domain file for our scenario can be found in the pddl folder of the directory. The corresponding task file can be generated using the method generate-pddl-problem (name domain objects init-predicates goal-predicates) from pddl-problem-generation.lisp in the lisp folder. 


Mockups
-------

The mockups package provides mockups scripts for all major components of the CaterROS project (excluding Knowledge) written in Python.

**Usage**

To start the mockups there are two launch files in the ``planning_launch`` package. You can start the mockups themselves with::

	roslaunch planning_launch mockups.launch

If you want to use the knowledge base, use::

	roslaunch planning_launch mockups_w_knowledge.launch

It can happen that the ``tf_subscriber`` node fails to launch properly when launching latter the first time. If this happens, just relaunch it and it should be fine.

You can only run plans if you launch with knowledge, because every plan needs to query the knowledge base. The first launch file is only for testing purposes when implementing service or action calls for example. But with the knowledge base launched you can run any plan and check if the plans themselves can be run without errors.

Most of the mockups have some support for the ROS parameter server. The graspkard mockup can either always instantly return an error value of 0 or simulate a optimization process over a few seconds. And the perception publisher's objects can be altered as well. For more detailed information on the how just look at the code. It's pretty simple. 

Robot-specific Commands
-----------------------

.. note::
	Hmmm
	
PR2
___

.. note::
	Explain how reacting to feedback works for example.

Turtlebot
_________

The following will include how to setup the Turtlebot itself in order for it to be used with the PR2 as a ROSmaster, how to maintain the Tortugabot and also what the high-level interface includes. 


Tortugabot handling
___________________________

1. Turn on the Laptop on top of the Robot and log in. This might take a while to boot. Generally keep it plugged in whenever you are not driving the robot, so that it is always charged when you actually want to drive the robot around. 

2. Connect it to the correct wifi, which is the PR2WLAN24. 

	TROUBLESHOOTING: If you don't see the wifi icon, try to plug out and back in the USB cable, which connects the laser scanner to the laptop. If it doesn't reappear, reboot the laptop. If it is still gone, reboot again, but this time during the loading screen of ubuntu, when it says "configuring Network", plug out and in the USB connector again. After that, it should have wifi. 

3. We assume all the necessary packages are already set up (which would be tortugabot_brngup, amcl, move_base, navigate_map...) and that you have pulled the sut_tortugabot package. If you aren't sure, just go to a terminal and see if you can roscd into them.

4. Plug in the battery. 
	
	Note: Tortugabot1 has one battery while Tortugabot2 has two connected batteries, acting as one, since it has bigger motors.

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

5. open a terminal on the robot or connect to it via ssh. Type:
	
	pr2master

Which is an alias to set the PR2 as master. Then open byobu.

6. execute::
	
	roslaunch sut_tortugabot t1_complete.launch

This will launch internally (in summary)::

	t1_minimal_bringup.launch 			|| that brings up the motordriver (roboclaw), the laser (hokyo), joystic teleop, and a few other things. (a little chain of launch files)
	t1_amcl_laser_without_map.launch 	|| this is amcl aka. localization. Without the map server since the map is published by the PR2.
	t1_move_base_laser.launch 			|| brings up move_base so that the robot can drive around.


Done!

More Troubleshooting: 

If it cannot connect to Hokuyo or Roboclaw: try relaunching the launch file a few times, plugging in the battery in and out a few times. If that doesn't help, reboot, repeat untill it works. If not, call for help.

Note: In the package of sut_tortugabot, are many more launch files. Basically you can call amcl and move_base separately for debugging purposes, some files are being called by other files, so please, just keep them there.


The sut_tortugabot package
___________________________

This package basically holds everything that needs to be executed localy on the Turtlebot itself. Which means, it holds a slightly adjusted roboclaw_node, the necessary parameters for move_base, the current map of the lab which we used and very many launch files, which are all prefixed.

What does prefixing mean?

It means that if we want to use multiple Robots in the same network, with one common master, they will likely have several topics in common. And if we then go on and publish something on these topics, it will affect all robots at the same time. For example, if we publish something on the topic cmd_vel, all robots will start moving and it's likely that one of them will crash into something. We don't want that to happen, therefore we need to keep all the topics and nodes separated. Also, it makes identifying which robot runs what, much easier.  So now, instead of just having multiple topics called /cmd_vel, we have something like: /tortugabot1/cmd_vel and /tortugabot2/cmd_vel. The non prefixed topics are the ones of the PR2. But even though they are not prefixed, we can now tell them appart from the others. 

Changes to these launch files: 
These launch files are basically replicas of the basic tortugabot launch files, just that here, all the nodes and topics are prefixed and remapped so that everything the tortugabot does, has basically a prefix of tortugabot1 to it. This way, it won't clash. Also the Tortugabot has his own tf tree, which gets published to the the master on a regular basis, but slow frequency. 
Also we included some topic_tools nodes into some of these files, in order to remap and trottle down some of the publish rates of some of the turtle nodes. Otherwise it would flood the network with huge tf and laser scanner data, and loose localization in the process. 
Some of the values of amcl were also adjusted to the situation.

.. note:: maybe mention some of the amcl parameters

Changes to roboclaw: 
The changes here are minor. Basically the topics are hardcoded and do not accept the parameters for frames like odom and base_footprint, so we had to prefix them manually here. Also, we commented out one diagnostic updates line, since it was making the robot lag terribly. 

Changes to the costmap parameters for move_base:
There is an own folder which holds all the parameter .yamls for move_base. Some of these values got adjusted as well. Generally, a good reference for calibrating is this http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide 
One important point to mention is the sim_time within the local_planner parameters. Setting this wrong can result in the robot spinning rather then moving towards it's goal (quote from that tutorial, actually.)