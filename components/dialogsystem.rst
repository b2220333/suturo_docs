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

Ros-based Image Streaming
----------

This module is represented by the component **rosCamera** in the architecture and accessible at rosCamera.py_. It samples images from the upper 2D-camera of Pepper, converts them into ROS images  and publishes them over  the ROS Image topic. The ROS parameters of this nodes are accessible at dialog.launch_ and follow:

- **VIDEOMODE**: indicates the position of the target camera. Value is **local** for a pc-webcam or **remote** for a robot camera
- **PEPPERIP**: indicates the Ip address of Pepper
- **PEPPERPORT**: indicates the port, Pepper should be accessed through

.. _rosCamera.py: https://github.com/suturo16/pepper-dialog/blob/master/dialogsystem/nodes/rosCamera.py

.. _dialog.launch: https://github.com/suturo16/pepper-dialog/blob/master/dialogsystem/launch/dialog.launch



Face Recognition
----------

This module is represented by the component **rosfaceAnalyzer** in the architecture and accessible at faceAnalyzer.py_. It subscribes to ROS Image topic described above, detects faces based on Haar-like features described in haarcascadeFrontalfaceDefault.xml_, introduces the detected faces into the dataset at faces_ and the detections into the folder detection_. Then, it trains the classifier for face recognition  from time to time with the data from faces_. This module communicates with the *Dialog Manager* through suitably defined ROS messages, servers and actions accessible at dialogsystemMsgs_. The ROS parameters of this nodes are accessible at dialog.launch_ and follow:

- **CVRESET**: indicates whether the module should be reinitialized at start or not. Value is **off** for volatile mode(all data lost on stop) and **on** for permanent mode(the model is preserved even on stop)
- **PATH_TO_DATASET**: absolute path to dataset folder. Any image must be saved with the format *personName_id_instance.jpg*. Example: franklin_1_3.jpg is the third image of the person having Franklin as name and 1 as Identification number.
- **PATH_TO_DETECTOR**: absolute path to detection folder
- **CVTHRESHOLD**: under this threshold, the recognized face is accepted
- **CVRTHRESHOLD**: under this threshold, the recognized face is considered as resemblance. Over this threshold, the recognized face is ignored
- **CVFACEWINDOW**: the minimal size of the side of the square, a detected face can fit in
- **CVDIMENSION**:  length of the image after pca-based compression(number of dimensions retained)
- **CVNEIGHBOR**: minimal number of meaningfull objects that should be detected around a presumed detected face before the latter is accepted
- **CVSCALE**: for scaling images before applying the detector, because The detector was trained on fixed-size images
- **CVSCANFREQUENCY**: for consistent visualization, a moving average with a window of size CVSCANFREQUENCY over the image stream takes place
- **CVINSTANCEFREQUENCY**: maximal number of images to save per face when detected
- **CVIDIMENSIONDEFAULT**: default size of the side of the square, each detected face should fit in
- **CVIDIMENSION**: size of the side of the square, each detected face should fit in. Same as **CVIDIMENSIONDEFAULT**, but dynamic because inferred from the available sizes of faces in the dataset.

.. _faceAnalyzer.py: https://github.com/suturo16/pepper-dialog/blob/master/dialogsystem/nodes/faceAnalyzer.py

.. _haarcascadeFrontalfaceDefault.xml: https://github.com/suturo16/pepper-dialog/tree/master/dialogsystem/data/facerecognition

.. _faces: https://github.com/suturo16/pepper-dialog/tree/master/dialogsystem/data/facerecognition

.. _detection: https://github.com/suturo16/pepper-dialog/tree/master/dialogsystem/data/facerecognition

.. _dialogsystemMsgs: https://github.com/suturo16/pepper-dialog/tree/master/dialogsystem_msgs


Speech Recognition
----------


This module is represented by the component **rosSpeechRecognizer** in the architecture and accessible at sphinxAsr.py_. It sets the parameters of the pure c++ module **PocketSphinx** and starts it. **PocketSphinx** receives Speech from a Gstreamer-TCP-server, recognizes it and then publishes the result for further processing. It is accessible at continuous.cpp_ and was derived from CMUSphinx_.  The ROS parameters of this nodes are accessible at dialog.launch_ and follow:

- **ASRCWD**: path prefix to access **PocketSphinx**
- **MLLR**: base path to access the speaker adapter of the speech recognizer. Allows online adaptation to speaker
- **HMM**: base path to access the acoustic model of the speech recognizer
- **ASRPATH**: base path to access the speech recognizer's object file
- **TRESHOLD**: the decoded speech is only considered under this threshold
- **DATAPATH**: base path to access the dictionary and language models of the speech recognizer
- **NBTHREADS**: the number of instances of speech recognizer to execute simultaneously and then combine their results into a more accurate one. It allows an ensemble learning-based recognition 
- **BEAMSIZE**: only the **BEAMSIZE** best results from the **NBTHREADS** available  must be combined to get the final result
- **INDEX**: this parameter is a positive integer and is used for naming of dictionary and language models. Example: **NBTHREADS**=2 and **INDEX**=33,then the folder **DATAPATH** will contain the files pepper33.dic(dictionary model of first thread/instance), pepper33.lm, pepper34.dic, pepper34.lm(language model of second thread)
- **HOST**: IP address of the underlying computer
- **PORT**: port of the Gstreamer-TCP-server
- **RPCPORT**: port of the RPC server, the decoded speech will be sent to
 
.. _sphinxAsr.py: https://github.com/suturo16/pepper-dialog/blob/master/dialogsystem/nodes/sphinx_asr.py

.. _continuous.cpp: https://github.com/suturo16/pepper-dialog/blob/master/dialogsystem/CMU/cnodes/continuous.cpp    

.. _CMUSphinx: https://github.com/cmusphinx/pocketsphinx/blob/master/src/programs/continuous.c


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
