Tutorial: Den PR2 starten
================================

bashrc einrichten
--------------

Um sich mit dem PR2 verbinden zu können, muss man noch folgende Zeilen zu der bashrc hinzufügen
Eigene ip muss durch die eigene ip ersetzt werden.

.. code:: bash

    alias pr2a=192.168.102.60
    alias pr2b=10.68.0.2
    export ROS_MASTER_URI=http://pr2a.ai.loc:11311
    export ROS_IP=eigene ip
    export ROS_HOSTNAME=eigene ip



PR2 starten
--------------
Zuerst verbindet mit sich per ssh mit dem PR2.

.. code:: bash

    ssh caterros@pr2a

Mit robot claim beansprucht man den PR2 für sich.

.. code:: bash

    robot claim

Als nächstes startet man byobu, damit die Prozesse nicht beendet werden, falls man die Verbindung zum PR2 verliert.
In byobu kann man mit f2 ein neues Terminal erstellen und mit f3 und f4 wechselt man durch die Terminals durch.

.. code:: bash

    byobu

Nun startet man folgende launchfiles

.. code:: bash

    roslaunch /etc/ros/indigo/robot.launch
    roslaunch pr2_manipulation.launch
    roslaunch iai_maps iai_maps.launch  ;; wenn man die ursprüngliche Karte haben möchte
    roslaunch maps.launch		;; für unsere eigene, große Karte vom Lab


Anmerkung: Die Semantic Map stimmt von den Positionen der Frames der Objekte mit unserer Karte zwar scheinbar überein, aber es wurde noch nicht getestet.    

.. roslaunch ~/pr2_manipulation.launch
    
Bevor man Giskard startet, sollten die Motoren es PR2 ausgeschaltet werden.

.. code:: bash

    roslaunch graspkard pr2.launch


Für die Kinect muss man openni auf dem PR2b starten. Dies macht man am besten aus byobu.

.. code:: bash

    ssh pr2b
    roslaunch /etc/ros/indigo/openni_head.launch


PR2 Lokalisieren
-----------------
Wenn der PR2 noch nicht bzw. falsch lokalisiert ist, muss man in Rviz unter "Global Options" den "Fixed Frame" auf map setzten, sonst funktioniert das nicht. Dannach in Rviz in der oberen Toolbar "2D Pose Estimate" auswählen und dementsprechend in der Map in Rviz platzieren. Es hilf dabei ein PoseArray auf dem topic "/particlecloud" zu haben. Mit diesem kann man sehen, wo der Roboter denkt, dass er gerade ist. Dannach einfach mit dem Roboter etwas durch die Gegend fahren, bis die meisten Pfeile verschwinden. Das sollte ihn Lokalisieren.


PR2 beenden
--------------
Wenn man fertig ist, führt man diese Befehle aus, um die gestarteten Prozesse zu beenden und den PR2 freizugeben.

.. code:: bash

    robot stop
    robot release
