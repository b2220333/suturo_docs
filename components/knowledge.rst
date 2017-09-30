.. note:: 
    Zur Dokumentation: Die Dokumentation soll so geschrieben sein, dass sie später anderen eine gute Übersicht über unser System gibt und dessen Nutzung vereinfacht. Wir sollen alle Komponenten unseres Systems (alle Features, die es so gibt oder Teile der Architektur, z.B. welche Dateien was enthalten, welche Regler es gibt...) beschreiben. Dabei helfen auch Grafiken, wie die Komponenten untereinander verbunden sind. Die Beschreibung soll eher auf "Highlevel"-Ebene sein, d.h. was die Features ungefähr machen und wo man sie findet, eine API ist nicht notwendig. Jeder sollte die von ihm entwickelten Features nach Möglichkeit selbst beschreiben!
    Memo: Die Notiz am Ende bitte wieder entfernen =)

=============
Knowledge
=============

The knowledge modules are basically about storing information and making it accessable in an easy way. 
Part of these information surely are about the world and its objects. For robots it is crucial to use these information, especially about objects, in order to interact with them. Another part of the information stored is information emerging from a spoken dialog. Intentions about a dialog for example, are stored in the knowledge base. 


Installation
----------
To install the knowledge system to your workspace, you need clone the repository to the src folder of your workspace. 
The easiest way to do this, is to copy the following lines into your terminal and replace the path with your local path.
Make sure that you have created a workspace before executing the following commands.

.. code-block:: bash
    :caption: setup
	
	cd ~/suturo16/suturo_ws/src/
	git clone git@github.com:suturo16/knowledge.git
	cd ../
	catkin_make

World state
----------

The world state comprises of the important concepts.

1. Test


2. Zweiter Test

CaterROS Cafeteria Modelling
----------

Pepper and openEASE
----------

Prython
----------
