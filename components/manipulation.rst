.. note:: 
Zur Dokumentation: Die Dokumentation soll so geschrieben sein, dass sie später anderen eine gute Übersicht über unser System gibt und dessen Nutzung vereinfacht. Wir sollen alle Komponenten unseres Systems (alle Features, die es so gibt oder Teile der Architektur, z.B. welche Dateien was enthalten, welche Regler es gibt...) beschreiben. Dabei helfen auch Grafiken, wie die Komponenten untereinander verbunden sind. Die Beschreibung soll eher auf "Highlevel"-Ebene sein, d.h. was die Features ungefähr machen und wo man sie findet, eine API ist nicht notwendig. Jeder sollte die von ihm entwickelten Features nach Möglichkeit selbst beschreiben!
Memo: Die Notiz am Ende bitte wieder entfernen =)


=============
Manipulation
=============

<General description and purpose>


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
