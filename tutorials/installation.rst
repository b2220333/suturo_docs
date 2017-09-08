

Installation: Projekt und Dependencies einrichten
=================================================

Dieser Artikel führt die Schritte auf, die für das Aufsetzen unseres Projektes notwendig sind.


Step-by-step Guide
------------------

Vorausgesetzte Software: ROS-Indigo auf Ubuntu 14.04 (siehe http://wiki.ros.org/indigo/Installation/Ubuntu). Empfohlen wird das Paket ros-indigo-desktop-full.

Build-Prerequisites
^^^^^^^^^^^^^^^^^^^

Folgende Schritte müssen ausgeführt werden, damit unser Projekt baut:


    1. Installiere benötigte Packete::

         sudo apt-get install ros-indigo-catkin ros-indigo-rospack python-wstool

    2. Installiere rosjava über folgenden Befehl::

         sudo apt-get install ros-indigo-rosjava ros-indigo-rosjava-*
    
    3. Installiere PR2-Navigation::

         sudo apt-get install ros-indigo-pr2-navigation

oder alles auf einmal installieren::

  sudo apt-get install ros-indigo-catkin ros-indigo-rospack python-wstool ros-indigo-rosjava ros-indigo-rosjava-* ros-indigo-pr2-navigation


Workspace einrichten
^^^^^^^^^^^^^^^^^^^^
    
    1. Erstelle irgendwo einen Catkin-Workspace, z.B.::
       
         mkdir -p ~/suturo_ws/src


    2. Initialisiere den Workspace::
       
         cd ~/suturo_ws/src
         catkin_init_workspace


    3. Source den Workspace oder füge ihn gleich zur .bashrc hinzu::

        cd ~/catkin_init_workspace
        catkin_make # erstellt den devel-Ordner
        source ~/catkin_ws/devel/setup.bash

       oder::

        catkin_make 
        cd ~
        echo "source ~/catkin_ws/devel/setup.bash" >> .bashrc
        bash


.. note:: Achte darauf, dass rosjava und dein Workspace gesourced sind. Überprüfen kannst du dies mit "echo $ROS_PACKAGE_PATH".
          Wichtig ist hierbei, dass der Workspace nach rosjava in eurer .bashrc auftaucht.

Optional: Wiederholt das ganze, um einen zusätzlichen Workspace für Dependencies wie Knowrob und Cram einzurichten::
     
    mkdir -p ~/suturo_dep/src
    cd ~/suturo_dep/src
    catkin_init_workspace
    cd ..
    catkin_make 
    source ~/catkin_ws/devel/setup.bash
    cd ~
    echo "source ~/suturo_dep/devel/setup.bash" >> .bashrc
    bash


Klone die Repositories, die du nutzen möchtest, in den src-Ordner deines Workspaces. Es gibt knowledge, planning, perception, manipulation und suturo_msgs. Für das Ausführen des gesamten Projektes werden alle Repositories benötigt. 

Der entsprechende git-Befehl ist::

    git clone <URL>

Die entsprechenden URLs sind:

+--------------+------------------------------------------+----------------------------------------------+
| Name         | SSH                                      | HTTPS                                        |
+--------------+------------------------------------------+----------------------------------------------+
| planning     | git@github.com:suturo16/planning.git     | https://github.com/suturo16/planning.git     |
+--------------+------------------------------------------+----------------------------------------------+
| perception   | git@github.com:suturo16/perception.git   | https://github.com/suturo16/perception.git   |
+--------------+------------------------------------------+----------------------------------------------+
| knowledge    | git@github.com:suturo16/knowledge.git    | https://github.com/suturo16/knowledge.git    |
+--------------+------------------------------------------+----------------------------------------------+
| manipulation | git@github.com:suturo16/manipulation.git | https://github.com/suturo16/manipulation.git |
+--------------+------------------------------------------+----------------------------------------------+
| suturo_msgs  | git@github.com:suturo16/suturo_msgs.git  | https://github.com/suturo16/suturo_msgs.git  |
+--------------+------------------------------------------+----------------------------------------------+


Nun kannst du den Workspace bauen::
    cd ~/suturo_ws/src
    catkin_make


Zusätzlich benötigt für Perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. note:: TODO


Zusätzlich benötigt für Manipulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Giskard installieren
"""""""""""""""""""""""
Installationsanweisung auf https://github.com/suturo16/giskard/tree/feature/GiskardLanguage folgen. Zum Branch feature/GiskardLanguage wechseln. 


Zusätzlich benötigt für Knowledge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

SWI-Prolog installieren
"""""""""""""""""""""""

SWI-Prolog wird benötigt, damit die json_prolog Services aufgerufen werden können::

    sudo apt-get install swi-prolog swi-prolog-*

Vor der ersten Nutzung von Prolog kann es zu verschiedenen Fehlern kommen. Seht dazu den Artikel jpl Troubleshooting.

.. note:: Den Artikel gibt es noch nicht.


Umgebungsvariablen
""""""""""""""""""

a. Füge die JAVA_HOME und SWI_HOME_DIR Umgebungsvariablen hinzu::

    export JAVA_HOME=/usr/lib/jvm/default-java
    export SWI_HOME_DIR=/usr/lib/swi-prolog


b. Füge die Java- rdner zu LD_LIBRARY_PATH hinzu. Wähle den für dein System zutreffenden Befehl aus::

    # for amd_64 systems (64 bits):
    export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/amd64:/usr/lib/jvm/default-java/jre/lib/amd64/server:$LD_LIBRARY_PATH
     
    # for i386 systems (32bits):
    export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/i386:/usr/lib/jvm/default-java/jre/lib/i386/server:$LD_LIBRARY_PATH

c. Optional: Prolog-History:
   "It is further recommended to add the following to your ~/.plrc file (create it if it does not exist). This will give you a global command history for the Prolog shell, which is very convenient when you have to repeatedly restart Prolog during testing and debugging." ::

        rl_write_history :-
          expand_file_name("~/.pl-history", [File|_]),
          rl_write_history(File).

        :- (
          current_prolog_flag(readline, true)
         ->
          expand_file_name("~/.pl-history", [File|_]),
          (exists_file(File) -> rl_read_history(File); true),
          at_halt(rl_write_history)
         ;
          true
         ).

   Quelle und evtl. weitere Infos: http://www.knowrob.org/installation/workspace.


Knowrob installieren
""""""""""""""""""""

Erstelle einen neuen catkin workspace ::

		mkdir -p ~/<wsname>/src

Prüfe maven deployment path ::

		echo $ROS_MAVEN_DEPLOYMENT_REPOSITORY

Sicherstellen, dass der path leer ist ::

		export ROS_MAVEN_DEPLOYMENT_REPOSITORY=""

Aktualisiere dependencies ::

		rosdep update

In den neuen workspace und wstool directory aufsetzen ::

		cd ~/<wsname>/src
		wstool init
		wstool merge https://raw.github.com/knowrob/knowrob/master/rosinstall/knowrob-base.rosinstall

.rosinstall um iai-common-msgs erweitern. ::

		gedit .rosinstall

Das entsprechende Repo, also diesen Text, in die .rosinstall kopieren ::

		- git:
   			local-name: iai_common_msgs
   			uri: https://github.com/code-iai/iai_common_msgs.git

Pakete ziehen ::

		wstool update

Rosdep Dependencies bauen ::

		rosdep install --ignore-src --from-paths stacks/

Knowrob bauen ::

		cd ..
		catkin_make

Knowrob sourcen ::

		source devel/setup.bash

Am besten danach im SUTURO Workspace den build und devel Ordner löschen und neu bauen, damit sie setup.bash vom SUTURO Workspace auch die knowrob source enthält.

**Troubleshooting**

* Falls knowrob_vis das Symbol setTop() nicht findet, Gradle build löschen und neu bauen. ::

		cd ~/<wsname>/src/stacks/knowrob/knowrob_vis
		./gradlew clean
		cd ~/<wsname>
		catkin_make

* Maven Dependency beim bauen reparieren
   *  Bei einem Error im Build suchen, wohin die vorher gebauten Pakete per Maven kopiert wurden.
   *  Ist dort nicht der aktuelle Workspace angegeben steht in ROS_MAVEN_DEPLOYMENT_REPOSITORY wahrscheinlich der Path, in den fälschlicherweise kopiert wurde. ::

   			echo $ROS_MAVEN_DEPLOYMENT_REPOSITORY

   *  Die Variable ROS_MAVEN_DEPLOYMENT_REPOSITORY muss geleert werden ::

			export ROS_MAVEN_DEPLOYMENT_REPOSITORY=""

   *  ROS_MAVEN_DEPLOYMENT_REPOSITORY ist jetzt nur in der aktuellen Terminal-Session leer. In dem aktuellen Terminal sollte dann nochmal versucht werden zu bauen. Da Knowrob eigentlich nur ein mal gebaut werden muss reicht das. Wenn man aber Knowrob regelmäßig bauen möchte sollte man herausfinden ob es wichtig ist, dass in ROS_MAVEN_DEPLOYMENT_REPOSITORY der Path steht, den wir gerade gelöscht haben.
   
   
Zusätzlich benötigt für Planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

EMACS installieren
"""""""""""""""""""
Installiert folgendes, um Lisp in Emacs ausführen zu können::

    sudo apt-get install ros-indigo-roslisp-repl

CRAM installieren
"""""""""""""""""""""""
Für Planning wird die minimale Installation von Cram benötigt. Dafür im src-Verzeichnis des Dependency-Workspaces die folgenden Befehle ausführen: 

$ git clone https://github.com/cram2/cram_3rdparty.git
$ git clone https://github.com/cram2/cram_core.git
$ rosdep install --ignore-src --from-paths cram_3rdparty cram_core
$ cd .. && catkin_make
(siehe http://www.cram-system.org/installation)


Downwards Planner für den Plangenerator installieren
""""""""""""""""""""""""""""""""""""""""""""""""""""""

Für den Plangenerator verwenden wir den Fast Downward Planer von http://www.fast-downward.org/.

Um diesen für den Plangenerator nutzbar zu machen, muss zunächst ein Ordner angelegt werden, z.B. "pddl" im Dependency-Workspace. In diesen wird eine Datei mit dem Namen "setup.py" eingefügt mit dem Inhalt: ::

	#!/usr/bin/env python

	from distutils.core import setup

	setup(name='pddl',
     	version='1.0',
     	description='pddl stuff',
     	author='someone',
     	author_email='someone@stuff.net',
     	url='https://www.python.org/sigs/distutils-sig/',
     	packages=['downward'],
    	) 


Die detaillierte Beschreibung zum Einrichten des Fast Downward Planers findet sich hier: http://www.fast-downward.org/ObtainingAndRunningFastDownward. 

Um das Vorhandensein aller nötigen Dependencies sicherzustellen, führe ::
	sudo apt-get install cmake g++ g++-multilib mercurial make python
aus. 

Danach clone das entsprechende Verzeichnis im vorher angelegten Ordner in den Ordner "downward" 

.. code:: bash

	cd pddl
	hg clone http://hg.fast-downward.org downward

Dann kann der Planer im Ordner "downward" gebaut werden: 

.. code:: bash

	cd downward
	./build.py

Nun erstellt die leere Datei "__init__.py" im "downwards"-Ordner. 
Gehe in den Unterordner driver und kommentiere in der Datei main.py die Zeile 

.. code:: python

	# sys.exit(exitcode)

aus, da der Server vom Plangenerator sonst nichts zurückliefern kann, wenn er aufgerufen wird. 

Nun führe in dem erstellen Ordner aus: 

.. code:: bash

	sudo pip install -e . 
Nun lässt sich der Planer von überall als Python-Paket importieren und der Plangenerator erhält Zugriff auf ihn. 
