Tutorial: Start the PR2
================================

bashrc setup
--------------

Everybody who wants to connect to the PR2 needs to add the folloging lines to his/her bashrc.

.. code:: bash

    alias pr2a=192.168.102.60
    alias pr2b=10.68.0.2
    export ROS_MASTER_URI=http://pr2a.ai.loc:11311
    export ROS_IP=<your-IP>
    export ROS_HOSTNAME=<your-IP>



PR2 start
--------------
First connect to the PR2 via **ssh**.

.. code:: bash

    ssh caterros@pr2a

With **robot claim** you get control over the robot. If anybody else has already claimed control over PR2, ask this person for permission first.

.. code:: bash

    robot claim

To provide an environment, where anybody can access the processes running on the PR2, we use **byobu**.
In byobu you can create a new window with **F2**, switch between windows with **F3** and **F4** and close  a window with **Ctrl-d**. *Please do* **NOT** *call byobu inside of a byobu session!*

.. code:: bash

    byobu

Now start the main launchfiles of the PR2. Use **F2** to create a new window for each process.

.. code:: bash

    roslaunch /etc/ros/indigo/robot.launch
    roslaunch pr2_manipulation.launch
    roslaunch iai_maps iai_maps.launch  ;; wenn man die ursprüngliche Karte haben möchte
    roslaunch maps.launch		;; für unsere eigene, große Karte vom Lab

    
Make sure to disable the motors of PR2 before you launch the graspkard node. 

.. code:: bash

    roslaunch graspkard pr2.launch


To use the kinect for perception we need the **openni** node. We start that on the second computer, **PR2b**. For that we can ssh to pr2b from within byobu. This is ok, but a second byobu session isn't!

.. code:: bash

    ssh pr2b
    roslaunch /etc/ros/indigo/openni_head.launch
    
PR2 shutdown
--------------
When you are ready, kill all the processes in byobu and close each window with **Ctrl-d**. When shutting down the last window you should be in a common ssh connection with pr2a. Now you can stop the remaining/hidden/background processes and release you claim on the PR2.

.. code:: bash

    robot stop
    robot release
    
Launch the SUTURO nodes on Bernd
--------------------------------

Connect to our server Bernd with as the user ''caterros'' at IP ''192.168.100.234''. Launch byobu and start the following nodes. In the end there should be 5 channels in your byobu when you finish.

Launch Manipulation action server. Either localy on your mashine - if you have everything installed - or on Bernd:

    .. code:: bash
    
	roslaunch suturo_action_server pr2_action_server.launch

Launch Knowledge. Preferably localy, since when launched on Bernd it has a weird delay (info from 29.06.2017, remove when fixed.)

    .. code:: bash
    
	roslaunch knowledge_launch suturo_knowledge_full.launch

Launch Perception. Either on the Robot or on Bernd. (When last checked on 28.06.2017, it was working better on the PR2, but this might change.)

    .. code:: bash
    
	rosrun percepteros caterrosRun cateros.xml

Launch plan generator on Bernd.

    .. code:: bash
    
	rosrun plan_generator generate_plan_server.py

Launch peppers dialog system on Bernd.

    .. code:: bash
    
	~/pepperdialog/pepperdialog/launcher.sh

Nice to know
------------
- Don't let knowledge run for too long. After  about 20 mins it get's very slow and won't react to queries. In That case, in planning, no action can be executed and emacs will seem "stuck" at the same spot of execution. Just restard knowledge + percepion then.
- Generally: start knowledge, start perception, check if perception percieves things, wait till plate Axis z (blue) shows upwards, then shut down perception and start planning! Ofc. you also need to wait till a box is recognized, etc. 
- IP of Bernd: 192.168.100.234
- if you want to run rviz all the time but your pc is not the newest, you can put most of the workload for running rviz on the pr2b. For that you need to install (`vglconnect <https://sourceforge.net/projects/virtualgl/files/2.5.2/>`_.) Then you can run: 

	.. code:: bash
	
	vglconnect caterros@pr2b
	vglrun rosrun rviz rviz
	
done. You might not to reconfigure Rviz though a bit. But your PC will thank you!

PR2 localization
-----------------

If you want to (re)localize the PR2, you have to set your Rviz straight. In ''Global Options'' the ''Fixed Frame'' has to be set to ''map'', or else this won't work. After this you can use the button ''2D Pose Estimate'' at the top of the Rviz panel and set the position of the PR2 by clicking on the desired position and dragging the spawning arrow into the right direction. For further aid you can set the PoseArray topic to ''/partickecloud'', to see, where the PR2 thinks he is localized. The vizualization of the pose arrays is a huge mess right now. Just take the controller and drive the PR2 around a bit, this should refine the estimated pose.
