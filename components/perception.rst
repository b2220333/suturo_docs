.. note:: 
Zur Dokumentation: Die Dokumentation soll so geschrieben sein, dass sie später anderen eine gute Übersicht über unser System gibt und dessen Nutzung vereinfacht. Wir sollen alle Komponenten unseres Systems (alle Features, die es so gibt oder Teile der Architektur, z.B. welche Dateien was enthalten, welche Regler es gibt...) beschreiben. Dabei helfen auch Grafiken, wie die Komponenten untereinander verbunden sind. Die Beschreibung soll eher auf "Highlevel"-Ebene sein, d.h. was die Features ungefähr machen und wo man sie findet, eine API ist nicht notwendig. Jeder sollte die von ihm entwickelten Features nach Möglichkeit selbst beschreiben!
Memo: Die Notiz am Ende bitte wieder entfernen =)

=============
Perception
=============

<General description and purpose>


CaterrosRun
----------
Implemented classes: CaterrosRun, CaterrosControlledAnalysisEngine, CaterrosPipelineManager

How to execute: `rosrun percepteros caterrosRun caterros` 

This is a customized version of the robosherlock run functionality. It starts RoboSherlock while offering the possibility to change the executed Pipeline during runtime. A service is offered at percepteros/set_pipeline, which takes a list of object names for which it will then start a customized pipeline. Currently only one object is supported, this can however easily be expanded by adding appropriate pipelines.

Pipelines are defined as yaml files in the config folder. In order to let the system know a new pipeline exists, the name of the pipeline has to be added to the setPipelineCallback function in CaterrosPipelineManager, where object names are mapped to config files.

Initially, all Annotators have to be known to RoboSherlock. They can be defined in an analysis engine xml. The xml used in the SUTURO Project is caterros.xml.



CakeAnnotator
----------

...
----------
