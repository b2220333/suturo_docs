.. note:: 
      Zur Dokumentation: Die Dokumentation soll so geschrieben sein, dass sie später anderen eine gute Übersicht über unser System gibt und dessen Nutzung vereinfacht. Wir sollen alle Komponenten unseres Systems (alle Features, die es so gibt oder Teile der Architektur, z.B. welche Dateien was enthalten, welche Regler es gibt...) beschreiben. Dabei helfen auch Grafiken, wie die Komponenten untereinander verbunden sind. Die Beschreibung soll eher auf "Highlevel"-Ebene sein, d.h. was die Features ungefähr machen und wo man sie findet, eine API ist nicht notwendig. Jeder sollte die von ihm entwickelten Features nach Möglichkeit selbst beschreiben!
      Memo: Die Notiz am Ende bitte wieder entfernen =)

=============
Planning
=============

<General description and purpose>


pepper_communication
----------

plan_execution
----------

plan_generator
----------
The plan_generator module provides access to the classical planning system Fast Downward from http://www.fast-downward.org/ using a ROS service in python. It can be used to generate a plan for a given task within a given domain. In the case of the CaterROS café, it can be used to find a plan for the task of serving a given amount of pieces of cake in the CaterROS domain. Nevertheless, the underlying service can also be used for any other task and corresponding domain.

To use the plan generator for CaterROS, you have to: 
1. Follow the installation instructions at: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

2. Run the server for the python service 

      .. code:: bash

            rosrun plan_generator generate_plan.py

3. Start the demo as explanined at: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX



The Fast Downward planning system needs two inputs: a domain definition and a task definition written in the Planning Domain Definition Language (PDDL). You can find a good introduction on PDDL at: http://www.cs.toronto.edu/~sheila/2542/s14/A1/introtopddl2.pdf. 


provides a service that can be used to generate a plan for a given task within a given domain dynamically. The resulting plan is contained in a json string that can easily be transformed into a list of CRAM's action designators. 

In our implementation, the service is called within the plan_execution module. 

Fast Downward is based on the Planning Domain Definition Language (PDDL). The algorithm needs two files as input: a domain file and a task file. The domain file for our scenario can be found in the pddl folder of the directory. The corresponding task file can be generated using the method generate-pddl-problem (name domain objects init-predicates goal-predicates) from pddl-problem-generation.lisp in the lisp folder. 




planning_common
----------

planning_communication
----------

planning_launch
----------

pr2_command_pool
----------


sut_mockups
----------



turtle_command_pool
----------
