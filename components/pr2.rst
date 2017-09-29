PR2
===

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
    roslaunch iai_maps iai_maps.launch

    
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
