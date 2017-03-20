

Installation: Start ueber Pepper
=================================

In diesem Tutorium wird zum einen beschrieben, wie Pepper- und PR2-Roboter miteinander verbunden werden. Zum anderen lernt man, wie das Sprachdialogsystem von Pepper gestartet wird.


Physikalische Verbindung Pepper-PR2
-----------------------------------

Da eine lokale(vom Roboter aus) Ausführung des Sprachdialogsystems ungeheuer starke Änderungen an dem auf dem Roboter vorliegenden Grundsystem fordern würde und erst seit ein Paar Monaten  laut Aldebarans Ingenieure möglich ist, wird eine Fernausführung(vom Ferncomputer aus) von Anwendungen empfohlen. Aus diesem Grund wird das Sprachdialogsystem von Pepper auf einem Ferncomputer(Proxy) ausgeführt, welcher über ein lokales Netzwerk(WiFi/Ethernet) sowohl mit PR2 auch mit Pepper verbunden ist.

Installation und Einstellungen von Programmen
---------------------------------------------

Betriebsystem
^^^^^^^^^^^^^

Auf dem Proxy-Computer muss ein Betriebsystem Ubuntu 14.04 LTS 64bits installiert werden::

     http://releases.ubuntu.com/14.04/


Ros-System
^^^^^^^^^^^

Auf dem Proxy-Computer muss ein Ros-Indigo-System installiert werden::

    http://wiki.ros.org/indigo/Installation/Ubuntu


Catkin-Tools
^^^^^^^^^^^^^

Zum Kompieleren von Ros-Paketen müssen die Catkin-Tools auch installiert werden::

    http://catkin-tools.readthedocs.io/en/latest/installing.html


Python
^^^^^^^^^^^

Auf dem Proxy-Computer muss eine Umgebung python2.7 Ubuntu 64bits installiert werden::

    https://wiki.ubuntuusers.de/Python/

Prüfen Sie die Installation::
 
   $python --version
   Python 2.7.6


Installation vom Paket naoqi_bridge_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mit diesem Paket können sich Ros-Programme und native NAOqi-Programme Daten austauschen. NAOqi ist das Betriebsystem auf Pepper.

     1. Starten Sie den das Shell-Programm und legen Sie einen Ordner "test" (oder etwas anderes) an::

        $mkdir test

     2. Im Ordner test Legen Sie einen Ordner src an::

        test$mkdir src

     3. Im Ordner src duplizieren Sie das Paket naoqi_bridge_msgs::

        src$git clone https://github.com/ros-naoqi/naoqi_bridge_msgs.git 

     4. Kompilieren Sie das Paket naoqi_bridge_msgs vom Ordner test aus::

        src$cd ..

        test$catkin build naoqi_bridge_msgs
        

Installation vom Pepper-Python-SDK
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Um Feraufrufe von NAOqi-Module(vom Proxy-Computer aus) durchzuführen, wird dieses Paket benötigt.

   1. Von der Aldebarans Webseite aus laden Sie das Pepper-Python-SDK für Linux 64 bits herunter::

      https://developer.softbankrobotics.com/. Momentan ist nur Version 2.5.5.5 Linux 64bits Python2.7 verfügbar

   2. Legen Sie einen Ordner naoqi im src Ordner::

      src$mkdir naoqi

   3. Kopieren und entpacken Sie das heruntergeladene Paket im Ordner naoqi::

      src$ cp ~/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz naoqi

      src$cd naoqi

      naoqi$tar xzf ~/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz

   4. Fügen Sie den SDK-Pfad in $PYTHONPATH hinzu::

      naoqi$echo 'export PYTHONPATH=~/naoqi/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages:$PYTHONPATH' >> ~/.bashrc

   5. Prüfen Sie die Installation::

      naoqi$python
      
      >>import naoqi
      
      >>
