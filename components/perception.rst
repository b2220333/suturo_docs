.. note::
    Zur Dokumentation: Die Dokumentation soll so geschrieben sein, dass sie später anderen eine gute Übersicht über unser System gibt und dessen Nutzung vereinfacht. Wir sollen alle Komponenten unseres Systems (alle Features, die es so gibt oder Teile der Architektur, z.B. welche Dateien was enthalten, welche Regler es gibt...) beschreiben. Dabei helfen auch Grafiken, wie die Komponenten untereinander verbunden sind. Die Beschreibung soll eher auf "Highlevel"-Ebene sein, d.h. was die Features ungefähr machen und wo man sie findet, eine API ist nicht notwendig. Jeder sollte die von ihm entwickelten Features nach Möglichkeit selbst beschreiben!
    Memo: Die Notiz am Ende bitte wieder entfernen =)

=============
Perception
=============

The purpose of perception in this project is to detect various objects and facts in or about the kitchen.

Tools for serving cake, the cake itself and the position of the robot in the kitchen are detected.

For data acquisition a kinect on the robots head and a laserscanner at the base of the robot are used.

Installation
----------
The framework used for detection is RoboSherlock, for which installation instructions can be found here: http://robosherlock.org/install.html

(Also needed is PCL 1.8, for which installation instructions can be found here: https://github.com/hsean/Capstone-44-Object-Segmentation/wiki/PCL-1.8:-Ubuntu-14.04-Installation-Guide)

To get the pipeline itself, one needs to clone the perception repo of the Caterros project: https://github.com/suturo16/perception

The normal pipeline can be launched with `roslaunch percepteros cateros.launch`.

The plate pipeline needs a RoboSherlock annotator that is currently only existing in a pull request here: https://github.com/RoboSherlock/robosherlock/pull/86


CaterrosRun
----------
Implemented classes: CaterrosRun, CaterrosControlledAnalysisEngine, CaterrosPipelineManager

How to execute: `rosrun percepteros caterrosRun caterros`

This is a customized version of the robosherlock run functionality. It starts RoboSherlock while offering the possibility to change the executed Pipeline during runtime. A service is offered at percepteros/set_pipeline, which takes a list of object names for which it will then start a customized pipeline. Currently only one object is supported, this can however easily be expanded by adding appropriate pipelines.

Pipelines are defined as yaml files in the config folder. In order to let the system know a new pipeline exists, the name of the pipeline has to be added to the setPipelineCallback function in CaterrosPipelineManager, where object names are mapped to config files.

Initially, all Annotators have to be known to RoboSherlock. They can be defined in an analysis engine xml. The xml used in the SUTURO Project is caterros.xml.


Annotators
----------

BoardAnnotator
----------
Uses results from the following annotators: CakeAnnotator.

Results in: Recognition annotation and pose annotation for board.

How to use: Is present in the pipeline for cake detection, which can be started by
`rosrun percepteros caterrosRun cake`


Changeable parameters (in perception/percepteros/descriptors/annotators/BoardAnnotator.xml):

None


The BoardAnnotator uses the pose of the cake to determine the possible places for the board (must be directly the cake) and then searches for circles in the vincinity.

ColorClusterer
----------
Uses results from the following annotators: PointCloudClusterExtractor.

Results in: Rack annotation for board, ToolAnnotations for ColorClusters found on the board.

How to use: Is present in the pipeline for knife detection, which can be started with
`rosrun percepteros caterrosRun knife`


Changeable parameters (in perception/percepteros/descriptors/annotators/ColorClusterer.xml):

minHue(Standard: 150) - Minimal hue value points must have to pass as belonging to the rack.

maxHue(Standard: 360) - Maximal hue value points can have to pass as belonging to the rack.

diffHue(Standard: 7) - Difference of hue values used in color clustering on the rack.

diffVal(Standard: 10) - Difference of value values used in color clustering on the rack.

diffDist(Standard: 0.01) - Cutoff value of point distance used in color clustering on the rack.

minPoints(Standard: 3000) - Minimal number of points of correct color to make a cluster a rack.

minCluster(Standard: 1000) - Minimal number of points for clusters found in color clustering on the rack.


The ColorClusterer checks all clusters if they have enough points of the rack color, and thus finds the rack.

The rack is enriched with a rack annotation for average surface normal.

The rack is then again clustered by color, to detect the tool clusters. Theses clusters are new and are thus added to the scene,
whereby they get an tool annotation with average hue and value.

KnifeAnnotator
----------
Uses results from the following annotators: ColorClusterer.

Results in: Recognition annotation and pose annotation for knife cluster.

How to use: Is present in the pipeline for knife detection, which can be started with
`rosrun percepteros caterrosRun knife`


Changeable parameters (in perception/percepteros/descriptors/annotators/KnifeAnnotator.xml):

minHue(Standard: 40) - Minimal hue value a tool cluster must have in order to be considered a knife.

maxHue(Standard: 70) - Maximal hue value a tool cluster can have in order to be considered a knife.


The KnifeAnnotator checks all tool clusters for the correct color of the knife (yellow) and calculates the right pose for the knife.


PlateAnnotator
----------
Uses results from the following annotators: PointCloudColorSegmentation, PrimitiveShapeAnnotation.

Results in: Recognition annotation and pose annotation for plates.

How to use: Is present in the pipeline for plate detection, which can be started with
`rosrun percepteros caterrosRun plate`


Changeable parameters (in perception/percepteros/descriptors/annotators/PlateAnnotator.xml):

minHue(Standard: 100) - Minimal hue value a cluster must have in order to be considered a plate.

maxHue(Standard: 360) - Maximal hue value a cluster can have in order to be considered a plate.


The PlateAnnotator checks all color clusters for detected circles, and tries to fit a second circle into the cluster.

If both circles are found and fulfill some criteria the cluster is assumed to be a plate.


CakeAnnotator
----------
Implemented classes: CakeAnnotator

Uses results from the following annotators: PointCloudClusterExtractor, ClusterColorHistogramCalculator

Requirement: Localized robot

Results in: Recognition annotation and pose annotation for boxes.

The core functionality of this module is to detect boxes of the color which is specified in the Annotator xml file. In order to classify an object as a box, there need to be 3 visible planes which satisfy a number of constraints. The biggest plane is found first, the 2 subsequent planes are each smaller than it's predecessor. Both of the smaller planes need to be perpendicular to the biggest plane, and the smallest plane also needs to be perpendicular to the second biggest plane. 

TODO Bild

Cakes are always assumed to be standing on a table which results in their z-axis pointing in the same direction as the z-axis of the map frame. Therefore the z-axis of the first plane is restricted to be the z-axis of the map.

SpatulaRecognition
----------

Implemented classes: SpatulaRecognition

Uses results from the following annotators: PointCloudClusterExtractor

Requirement: None

Results in: Recognition annotation and pose annotation for the spatula.

The core functionality of this module is to detect the spatula according to a parameter set. The Annotator works the following way: it analyses all the clusters and identifies the first cluster with sufficently close parameters as a spatula. The parameters are the eigenvalues from a 3D principal componant analysis and the hue value. The euclidean distance serves as a distance measure. As the only the handle of the spatula is recognized as a cluster, the first eigenvector and the up-vector of the scene serve as a basis for the axis computation.

ObjectRegionFilter
----------

Implemented classes: ObjectRegionFilter

Uses results from the following annotators: none

Requirement: located Robot

Results in: Filtered point cloud around specified object location.

This Annotator was designed to stabilize the perception of objects whose approximate location is known beforhand. It operates on the following parameters given in a specified yaml file:
regionID: this specifies for every pipeline initiation which region parameters should be used 
viewsToProcess: the name of the cas view to process although in its current state only point clouds with a pcl::PointXYZRGBA type are valid
region center: [x, y, z] coordinates of the region relative to the head_mount_kinect_rgb_optical_frame frame
range: the width of the region for each axis

ROSPublisher
----------
Message type:

Topic name:

The ROSPublisher advertises a topic on the ROS Network. On this topic it publishes all objects with a recognition annotation. Contained in the published message are the object pose as well as the name, type and dimensions as needed.

ChangeDetector
----------
The change detector is an experimental feature which calculates the changed clusters between two pointclouds. There are two different methods for this implemented in SzeneRecorder.cpp. The first Method uses Octrees and the pcl function OctreeChangeDetection to find voxels in the second Octree which were not present in the first Octree. The distance of the new Voxels is then checked against the distance in the first image in order to decide, whether a voxel is new because it was occluded or because it was added to the scene.
The second approach uses the depth images and applies a threshold to the difference between the two images. The contours of the resulting binary image are then used to calculate the clusters, which are again annotated to be newly added or formerly occluded.
