Tutorial: Den PR2 starten
================================

bashrc einrichten
--------------

Um sich mit dem PR2 verbinden zu können, muss man noch folgende Zeilen zu der bashrc hinzufügen
Eigene ip muss durch die eigene ip ersetzt werden.

::

    alias pr2a=192.168.102.60
    alias pr2b=10.68.0.2
    export ROS_MASTER_URI=http://pr2a.ai.loc:11311
    export ROS_IP=eigene ip
    export ROS_HOSTNAME=eigene ip



PR2 starten
--------------
Zuerst verbindet mit sich per ssh mit dem PR2.
::
    ssh caterros@pr2a

Mit robot claim beansprucht man den PR2 für sich.
::
    robot claim

Als nächstes startet man byobu, damit die Prozesse nicht beendet werden, falls man die Verbindung zum PR2 verliert.
In byobu kann man mit f2 ein neues Terminal erstellen und mit f3 und f4 wechselt man durch die Terminals durch.
::
    byubo

Nun startet man folgende launchfiles
::
    roslaunch /etc/ros/indigo/robot.launch
    roslaunch pr2_teleop teleop_joystick.launch
    roslauch iai_maps iai_maps
    roslaunch ~/pr2_manipulation.launch
    
Bevor man Giskard startet, sollten die Motoren es PR2 ausgeschaltet werden.
::
    roslaunch giskard_examples pr2.launch sim:=false


Für die Kinect muss man openni auf dem PR2b starten. Dies macht man am besten aus byobu.
::
    ssh pr2b
    roslaunch /etc/ros/indigo/openni_head.launch

PR2 beenden
--------------
Wenn man fertig ist, führt man diese Befehle aus, um die gestarteten Prozesse zu beenden und den PR2 freizugeben.
::
    robot stop
    robot release
