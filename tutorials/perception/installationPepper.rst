

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

Zum Kompilieren von Ros-Paketen müssen die Catkin-Tools auch installiert werden::

    http://catkin-tools.readthedocs.io/en/latest/installing.html


Python
^^^^^^

Auf dem Proxy-Computer muss eine Umgebung python2.7 Ubuntu 64bits installiert werden::

    https://wiki.ubuntuusers.de/Python/

Prüfen Sie die Installation::
 
   $python --version
   Python 2.7.6


Installation vom Paket naoqi_bridge_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mit diesem Paket können sich Ros-Programme und native NAOqi-Programme Daten austauschen. NAOqi ist das Betriebsystem auf Pepper.

     1. Starten Sie das Shell-Programm und legen Sie einen Ordner "test" (oder etwas anderes) an::

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

Um Fernaufrufe von NAOqi-Modulen(vom Proxy-Computer aus) durchzuführen, wird dieses Paket benötigt.

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

      naoqi$python::
      >>import naoqi::
      >>
   
   6. Quitieren Sie das Python-Shell-Programm::
      
      >>quit()

      src$naoqi


Prüfung der Installation aller benötigten Python-Pakete
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Zum Prüfen der Installation eines Python-Pakets "pythonPaket" führen Sie folgenden Kommando aus::

    $python
    >>import pythonPaket
    >>

Wäre das Paket nicht installiert worden, wäre der Import fehlgeschlagen. Jetzt prüfen Sie die Installation folgender Pakete::

    >>import  sys
    >>import  roslib
    >>import  naoqi_bridge_msgs
    >>import  naoqi
    >>import  SimpleXMLRPCServer
    >>import  xmlrpclib
    >>import  rospy
    >>import  std_msgs
    >>import  numpy
    >>import  socket
    >>import  fcntl
    >>import  struct
    >>import  time
    >>import  subprocess

Bei fehlenden Ros-Paketen(rosxxx) sollte noch die Installation von ROS-Indigo geprüft werden. Würde ein anderes Paket fehlen, sollte man versuchen, es mit folgendem Befehl zu installieren::

    $sudo pip apt-get install pythonpaket


Installation des Sprachdialogsystems
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Source: https://github.com/suturo16/perception.git [Branche: feature/pepper-dialogsystem].

     1. Setzen Sie den Shell-Arbeitsplatz(CWD) auf ~/test/src

     2. Laden Sie das oben gennante Repository herunter und verbinden Sie es an die oben gennante Branche::

        src$git clone https://github.com/suturo16/perception.git ::
        src$cd perception::
        perception$ git checkout feature/pepper-dialogsystem::


Installation von ChatScript
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Source: https://github.com/bwilcox-1234/ChatScript.git [Branche: master].

     1. Setzen Sie den Shell-Arbeitsplatz(CWD) auf ~/test/src/perception/dialogsystem

     2. Laden Sie das oben gennante Repository herunter und verbinden Sie es an die oben gennante Branche::

        dialogsystem$git clone https://github.com/bwilcox-1234/ChatScript.git
        
        dialogsystem$cd ChatScript
        
        ChatScript$ git checkout master

     3. Kopieren Sie die Datei filespepper.txt vom dialogsystem nach ChatScript/RAWDATA::

        dialogsystem$cp filespepper.txt ChatScript/RAWDATA 

     3. Kopieren Sie den Ordner PEPPER vom dialogsystem nach ChatScript/RAWDATA::

        dialogsystem$cp -r PEPPER ChatScript/RAWDATA 

     4. Setzen Sie den Shell-Arbeitsplatz(CWD) auf ~/test/src/perception/dialogsystem/ChatScript/BINARIES

     5. Starten Sie das Programm LinuxChatScript64 mit folgendem Befehl::

        BINARIES$./LinuxChatScript64 local
        
        >>Enter user name:

     6. Geben Sie "username" als Benutzername ein::
 
        >>Enter user name:username
        ...
        username>>

     7. Geben Sie den Befehl ":build pepper" zum Kompilieren von ChatScript-Programmen ein::

        username>>:build pepper
        ...
        PEPPER: ...

     8. Drücken Sie Ctrl+C zum Beenden von ChatScript

      
Systemeinstellung
^^^^^^^^^^^^^^^^^^^

Das System wird durch ein Paar Paremeter eingestellt. Diese Parameter werden, wie folgt, eingestellt.

    1. Öffnen Sie die Datei dialogsystem/launch/dialog.launch mit einem Texteditor(gedit...)

    2. In dieser Datei hält jedes Param-Tag eine Variable nämlich den Kennzeichner(name) und den Wert(value). Setzen Sie die Variable INTERFACE auf den Namen der Netzwerkschnittstelle, womit sich der Computer über das Netzwerk mit PR2 und PEPPER verbindet::

       Der Standardname ist eth0: Ethernet eth, erste Schnittstelle 0

    3. Die anderen Parameter werden entweder automatisch aktualisiert oder ändern nicht. Jedoch kann im Notfall eine manuelle Einstellung erfolgen.


Das Sprachdialogsystem kompilieren 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    1. Setzen Sie den Shell-Arbeitsplatz(CWD) auf ~/test

    2. Kompilieren Sie das Ros-Paket dialogsystem::
  
       test$catkin build dialogsystem

Jetzt ist das Sprachdialogsystem vollständig auf Ihrem Computer installiert und bereit zur Nutzung.


Start des Sprachdialogsystems 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    1. Setzen Sie den Shell-Arbeitsplatz(CWD) auf ~/test

    2. Registrieren Sie das Ros-Paket dialogsystem::
  
       test$source devel/setup.bash

    3. Starten Sie den Sprachdialogsystem mit folgendem Befehl::

       test$roslaunch dialogsystem dialog.launch


Beenden des Sprachdialogsystems 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

     1. Drücken Sie Ctrl+C zum Beenden des Sprachdialogsystems

Autor:Franklnn Kenghagho Kenfack
