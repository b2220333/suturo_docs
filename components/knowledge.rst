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

1. Temporal Parts of Objects
''''''''''''''''''''''''''''

When storing spatial information about objects, it is crucial to connect these information to a certain point in time because it is the only way to enable the knowledge base to reason over new and old data respectively. In the picture below you can see how temporal parts are structured in KnowRob.

.. figure:: temporal_parts.png  
    :alt: Temporal Parts in KnowRob
    :scale: 30%
    :align: center

Due to this concept it is possible for objects in KnowRob to have their attributes connected to a specific point in time. 

2. Physical Parts of Objects
'''''''''''''''''''''''''''''

To improve on the modelling concept of objects in KnowRob, physical parts were introduced. Objects now consist of subobjects. For example a cake spatula consists of two subobjects for its handle and for its supporting plane. This is useful because sometimes you want to grasp specific parts of an object. In case of the cake spatula you probably wanna grasp it at its handle, therefore it is easier to just lookup the pose of the handle instead of the object itself, which is defined as the center of the object. Despite the fact, that this way of representing objects is more plausible from a modelling side of view, it also makes it easier to store constants for specific offset values that belong to a physical part of an object. The illustration below provides an example of a cake spatula object in KnowRob.

.. figure:: physical_parts.png  
    :alt: Temporal Parts in KnowRob
    :scale: 30%
    :align: center





''''''''''''''''''''''''''''''''''''''


CaterROS Cafeteria Modelling
----------

Pepper and openEASE
----------

Prython
----------
