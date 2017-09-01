.. note:: 
Zur Dokumentation: Die Dokumentation soll so geschrieben sein, dass sie später anderen eine gute Übersicht über unser System gibt und dessen Nutzung vereinfacht. Wir sollen alle Komponenten unseres Systems (alle Features, die es so gibt oder Teile der Architektur, z.B. welche Dateien was enthalten, welche Regler es gibt...) beschreiben. Dabei helfen auch Grafiken, wie die Komponenten untereinander verbunden sind. Die Beschreibung soll eher auf "Highlevel"-Ebene sein, d.h. was die Features ungefähr machen und wo man sie findet, eine API ist nicht notwendig. Jeder sollte die von ihm entwickelten Features nach Möglichkeit selbst beschreiben!
Memo: Die Notiz am Ende bitte wieder entfernen =)


=============
Manipulation
=============

Welcome to the documentation page of the manipulation components used in the CaterROS project! On this page we provide installation instructions for the system, an overview over our components and instructions on how to use them.

Installation
------------
To install the manipulation system to you workspace, you need clone the repository to the source folder of your workspace. After that you can use `wstool` to install the needed dependencies.

.. code-block:: bash
    :caption: HTTPS

    cd ~/path_to_my_ws/src
    git clone https://github.com/suturo16/manipulation.git
    wstool merge manipulation/dependencies.rosinstall
    wstool update

If you have an ssh key setup for the use with GitHub, you can also use the ssh to clone the repository and its dependencies.

.. code-block:: bash
    :caption: SSH

    cd ~/path_to_my_ws/src
    git clone git@github.com:suturo16/manipulation.git
    wstool merge manipulation/ssh.rosinstall
    wstool update

All that's left to do now, is build your workspace once using catkin.

System Overview
----------
Overall the system provides the ability to use the constraint based motion frame work `giskard` to move the PR2 robot, as well as a variety of controllers needed for the CaterROS scenario. 

Collision Avoidance
----------
What does it do:
The collision avoidance tries do avoid collisions with point clouds. The point clouds are converted to octrees to accelerate this process. This is done by the Octomap_server package. After that the collision avoidance finds the closest cell of the octree to each link of the collision query. If this distance drops under a certain threshold it is maximised to avoid the collision.

Usage:
Additionally to the Actionserver you have to launch the octomap_server package. You can do that by running roslaunch suturo_action_server octomap_mapping.launch. The octomap_server package listens for point clouds on the topic */kinect_head/depth_registered/points*. This can be changed in the launch file. After the octomap_server was launched the Actionserver should automatically use the collision avoidance for controllers with collision queries.

Component 2
----------

...
----------
