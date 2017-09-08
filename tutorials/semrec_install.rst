Tutorial: How to install Semrec
===============================

Clone the followng repos:
-------------------------
	
+------------------------+----------------------------------------------------+--------------------------------------------------------+
| Name         	   	 | SSH                                                | HTTPS                                                  |
+------------------------+----------------------------------------------------+--------------------------------------------------------+
| semrec                 | git@github.com:code-iai/semrec.git                 | https://github.com/code-iai/semrec.git                 |
+------------------------+----------------------------------------------------+--------------------------------------------------------+
| semrec_plugins         | git@github.com:code-iai/semrec_plugins.git         | https://github.com/code-iai/semrec_plugins.git         |
+------------------------+----------------------------------------------------+--------------------------------------------------------+
| cram_beliefstate       | git@github.com:cram2/cram_beliefstate.git          | https://github.com//cram2/cram_beliefstate.git         |
+------------------------+----------------------------------------------------+--------------------------------------------------------+
| designator_integration | git@github.com:code-iai/designator_integration.git | https://github.com/code-iai/designator_integration.git |
+------------------------+----------------------------------------------------+--------------------------------------------------------+
| cram_plans  		 | git@github.com:cram2/cram_plans.git                | https://github.com/cram2/cram_plans.git                |
+------------------------+----------------------------------------------------+--------------------------------------------------------+


after this, just catkin_make. If it doesn't work, it is missing sr_plugin_knowrob you might need to clone:

+------------------------+----------------------------------------------------+--------------------------------------------------------+
| Name         	   	 | SSH                                                | HTTPS                                                  |
+------------------------+----------------------------------------------------+--------------------------------------------------------+
| sr_plugin_knowrob      | git@github.com:code-iai/sr_plugin_knowrob          | https://github.com/code-iai/sr_plugin_knowrob          |
+------------------------+----------------------------------------------------+--------------------------------------------------------+


if this doesn't work, and it is missing an libconfig.h++ file, do the following to install it ::

	sudo apt-get install libncurses5-dev automake autoconf libconfig++8-dev libjson0-dev


Usage of Semrec
^^^^^^^^^^^^^^^

1. add dependecy
	1. add cram_beliefstate as a dependency to the package.xml
	2. add cram-beliefstate as a dependency to the asd file.
	3. load the lisp package this dependency has been added to. 

2. run semrec 
.. code:: bash

	rosrun semrec semrec

3. load package
load the lisp package this is the dependency of

4. try it out!
try the following after starting a rosnode ::

	(progn

	;; toplevel plan
	(cram-language-implementation::named-top-level (:name :pizza-demo)

	   ;; settings for semrec
	   (beliefstate::register-owl-namespace "knowrob"
	"http://knowrob.org/kb/knowrob.owl#" cpl-impl::log-id)
	   (beliefstate::register-owl-namespace "cram_log"
	"http://knowrob.org/kb/cram_log.owl#" cpl-impl::log-id)

	   ;; logging context
	   (let ((log-node-id (beliefstate:start-node "PERCEIVE-OBJECT" nil)))
	     (beliefstate::annotate-resource "objectActedOn"
	(cram-designators:desig-prop-value object-designator :name) "knowrob")

	     ;; plan call
	     (recognize-object object-designator)

	     ;; annotations
	     (beliefstate:add-topic-image-to-active-node
	"/kinect2_head/hd/image_color_rect")
	     (beliefstate:stop-node log-node-id :success T))

	  ;; logs extraction
	  (beliefstate::set-experiment-meta-data
	    "performedInMap"
	"http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j" :type
	:resource :ignore-namespace t))
	;; files export
	(beliefstate:extract-files))

