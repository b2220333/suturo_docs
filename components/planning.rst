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

The turtle_command_pool package holds stuff.

.. note::
	Please elaborate here about the turtle package.

	
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

.. note::
	Explain how to execute plans.

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

.. note::
	Explain location designator usage and costmaps.

