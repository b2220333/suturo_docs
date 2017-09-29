Launching CaterROS
==================

Prerequisites
-------------

- You can launch the nodes on any machine you want as long as all of them share the same master (most likely the PR2).
- Of course to launch a specific system, you have to have that system installed on the machine. Check the system's documentation for installation instructions.
- Have the :doc:`pr2` and :doc:`turtlebot` up and running.


Launch the CaterROS nodes
-------------------------
.. note::
  If you want to start some or all of the system on a single machine (e.g. an server) you can use byobu to have simple control of the machine when connecting via ssh.

Launch Manipulation action server.

    .. code:: bash
    
	    roslaunch suturo_action_server pr2_action_server.launch

Launch Knowledge.

    .. code:: bash
    
	    roslaunch knowledge_launch suturo_knowledge_full.launch

Launch Perception.

    .. code:: bash
    
	    rosrun percepteros caterrosRun cateros.xml

Launch peppers dialog system.

    .. code:: bash
    
	    ~/pepperdialog/pepperdialog/launcher.sh

Launch plan generator.

    .. code:: bash
    
	    rosrun plan_generator generate_plan_server.py

Then start a REPL and start planning::

	roslisp_repl
	
In the REPL type::

	,
	r-l-s TAB RET
	plan_execution RET
	TAB RET
	,
	i-p TAB RET
	pexecution RET
	
Now you are in the plan_execution package and can start executing plans.


PR2 localization
-----------------

If you want to (re)localize the PR2, you have to set your Rviz straight. 
In ''Global Options'' the ''Fixed Frame'' has to be set to ''map'', or else this won't work. 
After this you can use the button ''2D Pose Estimate'' at the top of the Rviz panel and set the position of the PR2 by 
clicking on the desired position and dragging the spawning arrow into the right direction. 
For further aid you can set the PoseArray topic to ''/partickecloud'', to see, where the PR2 thinks he is localized. 
The vizualization of the pose arrays seems like a huge mess at first. Just take the controller and drive the PR2 around a bit, 
this should refine the estimated pose.


Nice to know
------------
If you want to run Rviz all the time but your machine is not the newest, you can put most of the workload for running rviz 
on the pr2b. For that you need to install (`vglconnect <https://sourceforge.net/projects/virtualgl/files/2.5.2/>`_.).

Then you can run: 

.. code:: bash
	
  vglconnect caterros@pr2b
  vglrun rosrun rviz rviz
	
Done. You might need to reconfigure Rviz a bit, though. But your machine will thank you!

