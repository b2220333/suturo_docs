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

Architecture
------------

.. note::
	Explain how and why we use CRAM (v2).
	List packages, describe what uses what and why.


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

.. note::
	How to and why use mockups.

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

