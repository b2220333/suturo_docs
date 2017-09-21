.. note:: 
    Zur Dokumentation: Die Dokumentation soll so geschrieben sein, dass sie später anderen eine gute Übersicht über unser System gibt und dessen Nutzung vereinfacht. Wir sollen alle Komponenten unseres Systems (alle Features, die es so gibt oder Teile der Architektur, z.B. welche Dateien was enthalten, welche Regler es gibt...) beschreiben. Dabei helfen auch Grafiken, wie die Komponenten untereinander verbunden sind. Die Beschreibung soll eher auf "Highlevel"-Ebene sein, d.h. was die Features ungefähr machen und wo man sie findet, eine API ist nicht notwendig. Jeder sollte die von ihm entwickelten Features nach Möglichkeit selbst beschreiben!
    Memo: Die Notiz am Ende bitte wieder entfernen =)

=============
Dialog System
=============

This software allows the Pepper robot to hold a human-like conversation in the SUTURO16 Project from the Institute of Artificial Intelligence -University of Bremen.
The task of the robot in the project consists in taking care of the clients in a Cafe. The robot welcomes the clients, informs them about the Cafe services, takes their orders, forwards them to the robot baker and informs the clients on the evolution of their requests. 



Architecture
----------

.. figure:: SDSarchitecture.png  
    :alt: Dialog System's Overview
    :scale: 50%
    :align: center
    
    *Fig1. Dialog System's Architecture*

As we can see from the above architecture, the Dialog System is highly heterogeneous from the os perspective. 

- **NAOqi OS**: operating system of the target robot(Pepper). Pure libraries were needed for robot control access.
- **ROS Indigo**: needed for a more efficient management of the Dialog System's components and their intercommunication. Moreover, this packaging of components into ros nodes allows the Dialog System to interact with ROS environments. Just to recall that ROS Indigo is a virtual OS running on top of Linux Ubuntu.
- **Linux Ubuntu 14.04 LTS**: needed to run critical components which could neither be completely built from scratch, neither be appropriately adapted according to a chosen OS in the deadlines of the project. They could only be appropriately adapted to the project without changing the target OS to run them.

Consequently, the simplest way to launch the Dialog system consists in installing and running it remotely on any Linux Ubuntu 14.04 LTS platform.

Face Recognition
----------

Ros-based Image Streaming
----------

Speech Recognition
----------

Gstreamer-based Audio Streaming
----------

Basic Awareness
----------

System Core
----------

ChatScript
----------

Speech Synthesis
----------

RPC-Client
----------

RPC-Server
----------

Parameter Update
----------

Utility
----------



Installation and Start
----------
