=============
Schnittstellen
=============

Im Folgenden finden sich die Definitionen der Schnittstellen der einzelnen Gruppen sowie übergreifender Module.


Perception
----------

Services
________

- /percepteros/set_pipeline(List<ObjectNames>) -> Liste mit Objekten für die keine Pipeline gestartet werden konnte
        Führt dazu, dass in Perception eine Pipeline generiert wird, die die spezifizierten Objekte sucht.

Topics
______

- percepteros/object_detection -> suturo_perception_msgs/ObjectDetection
        Gefundene Objekte werden auf diesem Topic gepublished.

Knowledge
---------


Prolog
______


- set_info(+Object, +[Info])
       Dieses Prädikat ermöglicht das Ablegen und Ändern von Informationen zu einem Objekt oder als Parametersatz.
        Ein erfolgreiches ablegen der Informationen wird mit True bestätigt.
       Object:         Das Objekt oder die Zuordnung der Information. Dies kann durch folgende Angaben erfolgen:
               ObjectInstance:         Knowrob Objektinstanz in der Form 'Knife_xYz'
               ObjectName:             rdf Parameter mit der Kante knowrob:nameOfObject und dem Objektnamen als String aus create_object_info() wäre dies der Name 'Knife1', Knife2' etc.
               ObjectType:             Knowrobtyp des Objektes welches erzeugt werden soll um die Daten abzulegen. Bsp. 'Knife'
       Info:           Die Informationen welche abgelegt werden sollen als [Bezeichnung, Wert] Listenelemente. Dies führt zu einen Aufruf des Prädikates ähnlich zu set_info('Knife42', [[xCoord, 1.0],[yCoord,2.0],[zCoord,5.0],[isDirty, true],...])
        
                        Ein Info-Wertepaar der Form [nameOfObject, Name] wird ebenfalls als Identifier benutzt und würde versuchen ein entsprechendes Objekt in der KB zu finden. Ist dieses noch nicht vorhanden, wird es angelegt.
                        
- get_info(+Variables, -Returns)
       Fragt beliebige Informationen ab die den in Variables gegeben Konditionen entsprechen.
       Bsp.aufruf wäre get_info([xCoord, [nameOfObject, 'Knife42'], isDirty], Returns) --> Antwort: -[[isDirty, true],[xCoord,1.0]].
       Variables:      Liste mit Konditionen als [Bezeichnung, Wert] e.g. [nameOfObject, 'Knife42'] und Abfragewerten wie xCoord, y Coord, typeOfObject, etc.

- seen_since(+Name, +FrameID, +Timestamp) -> True/False
        Wurde das Objekt mit Namen "Name" und der Frame-ID "FrameID" seit dem Timestamp "Timestamp" wieder gesehen?

- disconnect_frames(+ParentFrameID, +ChildFrameID)
        Trennt zwei Objekte mit den gegebenen Frames, so dass die zuvor konstante Transformation genutzt wird, um die neue "absolute" Position des Objektes zu berechnen und zu publishen.

- cap_available_on_robot(Capability, Robot)
       Kann genutzt werden um einen Roboter zu identifizieren mit der bestimmten Fähigkeit oder um die Fähigkeiten eines bestimten Roboters zu erfragen.

       Capability in der Form:
               srdl2cap:'AcousticPerceptionCapability'
               srdl2cap:'PerceptionCapability'
               srdl2cap:'ObjectRecognitionCapability'
               srdl2cap:'VisualPerceptionCapability'
               ...

       Robot in der Form:              
               pepper:'JulietteY20MP_robot1'
               oder
               pr2:'PR2Robot1'

Service
______    
        
- connect_frames_service(String ParentFrameID, String ChildFrameID)
        Typ: suturo_knowledgE_msgs/srv/ConnectFrames.srv
        Verbindet zwei Objekte mit den gegebenen Frames, so dass in TF eine konstante Transformation vom Parent zum Child gepublisht wird.

Manipulation
------------

Der Actionserver zur Bewegung des Roboters bekommt ein Ziel in Form einer Nachricht vom Typ *suturo_manipulation_msgs/MoveRobotActionGoal*. Diese setzt sich zusammen, aus einer Liste von Gelenken, die vom Controller benutzt werden sollen, einer Beschreibung des Controllers, dem Namen des Feedback-Wertes und eine Liste von Parametern.

.. code::
  :name: suturo_manipulation_msgs/MoveRobotActionGoal

  string[] controlled_joints
  string controller_yaml
  string feedbackValue
  suturo_manipulation_msgs/TypedParam[] params

Die Liste der Gelenke, der Name des Feedbacks und die Liste der Parameter sind abhängig vom gewählten controller.

Parameter
_________

Parameter werden in Form von *suturo_manipulation_msgs/TypedParam* übergeben. 

.. code::
  :name: suturo_manipulation_msgs/MoveRobotActionGoal

  uint8 DOUBLE=0
  uint8 TRANSFORM=1
  uint8 ELAPSEDTIME=2
  bool isConst
  uint8 type
  string name
  string value

Da es nicht möglich ist, generische Nachrichtentypen zu bauen, bzw. solche schwierig zu debuggen wären, werden alle Daten als Strings kodiert und ihr Typ mittels enumerierten Werten im Attribut **type** der Nachricht festgehalten. Der Name des Parameters, welcher im Attribut **name** vermerkt wird, dient eigentlich nur dem Debugging. Die einzige Ausnahme stellen folgende Namen dar:

  - **r_gripper_effort**: Setzt immer die Griffstärke des rechten Greifers
  - **l_gripper_effort**: Setzt immer die Griffstärke des linken Greifers

Bei den Parametern wird zwischen konstanten und dynamischen Parametern unterschieden. Dynamische Parameter werden während der Ausführung des Controllers fortlaufend aktualisiert. Ob ein Parameter konstant oder dynamisch ist, wird über das Attribut **isConst** festgehalten. 

Unterstützte Typen:
 
 - **double**

     + *konstant*: Eine Zahl mit oder ohne "."

 - **transform**

     + *konstant*: Sieben durch Leerzeichen getrennte **double**, wobei die ersten drei die Position, die nächsten drei eine Rotationsachse und die letzte eine Rotation um diese Achse in Radianten beschreiben. Beispiel: "0 0 0 1 0 0 0" für die Identitätstransformation.
     + *dynamisch*: Zwei Namen von Frames die im TF-Baum existieren. Der erste Name ist dabei der des gesuchten Frames, der zweite der des Frames, zu dem der erste relativ bestimmt werden soll. 
       Beispiel: "glass table" um den Frame *glass* relativ zu *table* bestimmen zu lassen.

 - **elapsed time**

     + Gibt die Zeit seit Start des Controllers in Sekunden an. Alle Felder dieses Parameters werden ignoriert.
       

Die Reihenfolge der Parameter ist wichtig, da diese der in den Controllern entsprechen muss. Eine Ausnahme stellen dabei die fest benannten Parameter, welche gesondert behandelt werden und die Reihenfolge nicht beeinflussen.


Controller
__________

graspkard/gripper_control.yaml
""""""""""""""""""""""""""
    
    :Description: Changes the gripper opening width to the desired value.
    :Joint list: 
      - *graspkard/config/pr2_right_gripper.yaml*: Right gripper
      - *graspkard/config/pr2_left_gripper.yaml*: Left gripper
    :Parameter:
      - **double**: Desired gripper opening width *m*
    :Feedback: *feedback* The closer to :math:`0` the better.
    :Example parameters:
      - *graspkard/test_params/grasp_l_50.yaml* Grasping with the left gripper with 50% strength
      - *graspkard/test_params/grasp_l_50.yaml* Grasping with the right gripper with 50% strength
      - *graspkard/test_params/release_l_50.yaml* Releasing the left gripper with 50% strength
      - *graspkard/test_params/release_l_50.yaml* Releaseing the right gripper with 50% strength

graspkard/pr2_grasp_control_r.yaml
""""""""""""""""""""""""""
    
    :Description: Grasps a cylinder with the right arm. Opens the gripper in the process.
    :Joint list: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, right arm and gripper
    :Parameter:
      - **transform**: Frame of the cylinder in the reference frame of the Robot. This is *base_link* for the PR2.
      - **double**: Diameter of the cylinder in *m*
      - **double**: Hight of the cylinder in *m*
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameters:
      - *graspkard/test_params/approach_cylinder_r.yaml*: Moves the right arm to a cylinder with the name *cylinder* with the dimensions 5x14 *cm*. 

graspkard/pr2_grasp_control_l.yaml
""""""""""""""""""""""""""
    
    :Description: Grasps a cylinder with the left arm. Opens the gripper in the process.
    :Joint list: 
      - *graspkard/config/pr2_upper_body_left_arm.yaml*: Torso, right arm and gripper
    :Parameter:
      - **transform**:  Frame of the cylinder in the reference frame of the Robot. This is *base_link* for the PR2.
      - **double**: Diameter of the cylinder in *m*
      - **double**:Hight of the cylinder in *m*
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameter:
      - *graspkard/test_params/approach_cylinder_l.yaml*: Moves the right arm to a cylinder with the name *cylinder* with the dimensions 5x14 *cm*. 
        
graspkard/pr2_upper_body_joint_control.yaml
""""""""""""""""""""""""""
    
    :Description: Moves the joints of the torso into a goal position 
    :Joint list: 
      - *graspkard/config/pr2_upper_body.yaml*: Torso, both arms, no grippers
    :Parameter:
        - **double** Position des Gelenks *torso_lift_joint* in *m*
        - **double** Position des Gelenks *l_shoulder_pan_joint* in *rad*
        - **double** Position des Gelenks *l_shoulder_lift_joint* in *rad*
        - **double** Position des Gelenks *l_upper_arm_roll_joint* in *rad*
        - **double** Position des Gelenks *l_elbow_flex_joint* in *rad*
        - **double** Position des Gelenks *l_forearm_roll_joint* in *rad*
        - **double** Position des Gelenks *l_wrist_flex_joint* in *rad*
        - **double** Position des Gelenks *l_wrist_roll_joint* in *rad*
        - **double** Position des Gelenks *r_shoulder_pan_joint* in *rad*
        - **double** Position des Gelenks *r_shoulder_lift_joint* in *rad*
        - **double** Position des Gelenks *r_upper_arm_roll_joint* in *rad*
        - **double** Position des Gelenks *r_elbow_flex_joint* in *rad*
        - **double** Position des Gelenks *r_forearm_roll_joint* in *rad*
        - **double** Position des Gelenks *r_wrist_flex_joint* in *rad*
        - **double** Position des Gelenks *r_wrist_roll_joint* in *rad*
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameter:
      - *graspkard/test_params/upper_body_praying_mantis.yaml*: The *Praying Mantis* pose

graspkard/pr2_right_arm_joint_control.yaml
""""""""""""""""""""""""""
    
    :Description: Moves the joints of the right arm into a goal position. Even though the gripper is included in the joint list and parameters, it is not comtrolled.
    :Joint list: 
      - *graspkard/config/pr2_right_arm.yaml*: Right arm and gripper
    :Parameter:
        - **double** Position des Gelenks *r_shoulder_pan_joint* in *rad*
        - **double** Position des Gelenks *r_shoulder_lift_joint* in *rad*
        - **double** Position des Gelenks *r_upper_arm_roll_joint* in *rad*
        - **double** Position des Gelenks *r_elbow_flex_joint* in *rad*
        - **double** Position des Gelenks *r_forearm_roll_joint* in *rad*
        - **double** Position des Gelenks *r_wrist_flex_joint* in *rad*
        - **double** Position des Gelenks *r_wrist_roll_joint* in *rad*
        - **double** Position des Greifers in *m* - wird ignoriert
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameter:
      - *graspkard/test_params/r_arm_praying_mantis.yaml*: The *Praying Mantis* pose of the right arm

graspkard/pr2_left_arm_joint_control.yaml
""""""""""""""""""""""""""
    
    :Description: Moves the joints of the left arm into a goal position. Even though the gripper is included in the joint list and parameters, it is not comtrolled.
    :Joint list: 
      - *graspkard/config/pr2_left_arm.yaml*: Left arm and gripper
    :Parameter:
        - **double** Position des Gelenks *l_shoulder_pan_joint* in *rad*
        - **double** Position des Gelenks *l_shoulder_lift_joint* in *rad*
        - **double** Position des Gelenks *l_upper_arm_roll_joint* in *rad*
        - **double** Position des Gelenks *l_elbow_flex_joint* in *rad*
        - **double** Position des Gelenks *l_forearm_roll_joint* in *rad*
        - **double** Position des Gelenks *l_wrist_flex_joint* in *rad*
        - **double** Position des Gelenks *l_wrist_roll_joint* in *rad*
        - **double** Position des Greifers in *m* - wird ignoriert
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameter:
      - *graspkard/test_params/l_arm_praying_mantis.yaml*: The *Praying Mantis* pose of the left arm

graspkard/pr2_place_control_r.yaml
""""""""""""""""""""""""""
    
    :Description: Places a cylinder in a traget area with the right arm.
    :Joint list: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, right arm and gripper
    :Parameter:
      - **transform** Frame of the target area *base_link*.
      - **transform** Frame of the cylinder in *r_wrist_roll_link*.
      - **double** Diameter of the cylinder
      - **double** Height of the Cylinder
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameter:
      - *graspkard/test_params/place_cylinder_r.yaml*: Places a cylinder called *cylinder* in a target area called *goal_area*

graspkard/pr2_place_control_l.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Places a cylinder in a traget area with the left arm.
    :Joint list: 
      - *graspkard/config/pr2_upper_body_left_arm.yaml*: Torso, left arm and gripper
    :Parameter:
      - **transform** Frame of the target area *base_link*.
      - **transform** Frame of the cylinder in *r_wrist_roll_link*.
      - **double** Diameter of the cylinder
      - **double** Height of the Cylinder
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameter:
      - *graspkard/test_params/place_cylinder_l.yaml*: Places a cylinder called *cylinder* in a target area called *goal_area*


graspkard/knife_grasp.yaml - Messer greifen
"""""""""""""""""""""""""
    :Description: Grasps a knife with the right arm.
    :Joint list:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, left arm and gripper
    :Parameter:
      - **transform** Frame of the knife in *base_link*
      - **double** Height of the knife in *m*
      - **double** Length of the knife hanfle in *m*
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameter: TODO


graspkard/TODO - Messer umgreifen
"""""""""""""""""""""""""
    :Description: Messer sitzt beim ersten Greifen ungeeignet für das Schneiden im Greifer und wird mit Hilfe dieses Controllers in eine geeignete Position gebracht.
    :Joint list:
      - *graspkard/config/pr2_upper_body_grippers.yaml*: Torso, rechter Arm, linker Arm, rechter Greifer, linker Greifer
    :Parameter:
      - **transform** Frame des Messers in *base_link*
      - **double** Länge des Messers in *m*
      - **double** Länge des Griffes in *m*
      - **double** Höhe des Griffes in *m*
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Example parameter: TODO


graspkard/pr2_cut_r.yaml
"""""""""""""""""""""""""
    :Beschreibung: Cuts a cake parallel to its YZ-plane with the right arm.
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, left arm and gripper
    :Parameter:
      - **transform** Frame of the cake in *base_link*
      - **double** Length of the cake (X-dimension)
      - **double** Width of the cake (Y-dimension)
      - **double** Depth of the cake (Z-dimension)
      - **transform** Frame of the cake in *r_wrist_roll_link*
      - **double** Height of the knife
      - **double** Length of the knife handle
      - **double** Width of the piece of cake
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Beispiel-Parameter: *graspkard/test_params/cut.yaml*: Cuts a 1,5cm wide piece of cake *cake* with a knife *knife*.


graspkard/pr2_cut_position_r.yaml
"""""""""""""""""""""""""
    :Beschreibung: Moves the right arm into position for cutting the cake
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, right arm and gripper
    :Parameter:
      - **transform** Frame of the cake in *base_link*
      - **double** Length of the cake (X-dimension)
      - **double** Width of the cake (Y-dimension)
      - **double** Depth of the cake (Z-dimension)
      - **transform** Frame of the knife in *r_wrist_roll_link*
      - **double** Height of the knife
      - **double** Länge des Messergriffs
      - **double** Width of the piece of cake
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Beispiel-Parameter: *graspkard/test_params/cut_pos.yaml*: Moves the right arm into position for cutting a 1,5cm wide piece of cake from the cake *cake* with a knife *knife*.


graspkard/pr2_detatch_knife_r.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Detaches an object from the rack with the right arm. The Y-axis must point towards the rack. That way it is possible to use the last pose of the knife as rack pose.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, right arm and gripper
    :Parameter:
      - **transform** Frame of the knife io *r_wrist_roll_link*.
      - **transform** Frame of the rack in *base_link*.
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Beispiel-Parameter:
      - TODO
        
graspkard/pr2_look_at.giskard
""""""""""""""""""""""""""
    
    :Beschreibung: Points the RGB-Sensor of the Kinect at the center of a frame.
    :Gelenklisten: 
      - *graspkard/config/pr2_lookAt_joints.yaml*: Torso, Neigungs- und Drehgelenk
    :Parameter:
      - **transform** Frame to look at in *base_link*.
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Beispiel-Parameter:
      - *graspkard/test_params/poi_test.yaml*
        
graspkard/pr2_grasp_plate_r.giskard
""""""""""""""""""""""""""
    
    :Beschreibung: Uses the right arm to grasp a circular edge. The center of the edge is passed as frame. The Z-axis is the rotational axis. The angle between the outer edge and the rotational axis is also passed. SO it it usually greater than 90°.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, right arm and gripper
    :Parameter:
      - **transform** Frame of the center of the edge in *base_link*.
      - **double** Radius of the edge in *m*.
      - **double** Upper Z-Coordinate of the edge in the center frame.
      - **double** Width of the dge in *m*.
      - **double** Angle between the edge and the rotational axis in *rad*.
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Beispiel-Parameter:
      - *graspkard/test_params/pr2_grasp_plate_r.yaml*

graspkard/pr2_release_r.giskard
""""""""""""""""""""""""""
    
    :Beschreibung: Releases an object that was grasped with the right gripper. The gripper is moved backwards 12cm along the X-axis. The rotation does not change in the meantime. In order for this to work the starting position of the gripper must be passed as constant frame. Aditionally the opening width for the gripper must be passed.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, right arm and gripper
    :Parameter:
      - **transform** Starting frame of the gripper in *base_link*.
      - **double** Opening width of the gripper in *m*.
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Beispiel-Parameter:
      - Derzeit keine
        
graspkard/pr2_move_and_flip_r.giskard
""""""""""""""""""""""""""
    
    :Beschreibung: It is used to drop an object in a target area from a plate or spatula. Required are the transformation of the spatula relative to the right gripper, the frame of the target area, the width of the spatula and the radius of the target area.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, right arm and gripper
    :Parameter:
      - **transform** Spatula relative to *r_wrist_roll_link*.
      - **transform** target area in *base_link*.
      - **double** Width of the spatula in *m*.
      - **double** Radius of the target area in *m*.
    :Feedback: *feedback* the closer to :math:`0` the better.
    :Beispiel-Parameter:
      - *graspkard/test_params/move_and_flip.yaml*

Planning
----------
Auch, wenn Funktionen wie *cutCake()* intern keine Parameter benötigen, muss für die Kommunikation von Python zu Lisp mindestens ein Parameter in der Signatur angefragt werden. Das Aufrufen von Funktionen ohne Parameter ist von Python zum Lisp-RPC-Server nicht möglich. 

.. code::
  
  - RPC-Server
        - updateObserverClient(clientID, host, port)
            Der RPC-Server verwaltet eine Map von Clients und deren IPs/Ports. Bekommt er diese Anfrage updatet er die Infos des entsprechenden Clients oder legt ihn neu an.
        
        - cutCake(status)
            Um den Plan zum Kuchen schneiden anzustoßen. Soll sofort zurückgeben, wie lange das etwa dauern wird (also z.B. wie viele Aufträge vorher noch ausgeführt werden müssen). Return -1 bei serverseitigem Fehler.
            
        - stressLevel(status)
            Gibt die Auslastung des Servers als numerischen Wert zurück. Entspricht der Anzahl der Aufgaben, die noch durchzuführen sind.
            
        - do(task)
           Führt die gegebene Aufgabe **task** aus.
                       
        - assertDialogElement(json-string)
           Sendet das JSON an die Knowledgebase. Das Format ist hier https://docs.google.com/document/d/1wCUxW6c1LhdxML294Lvj3MJEqbX7I0oGpTdR5ZNIo_w definiert.
        
        - getCustomerInfo(customer-id)
           Liefert die Info zum Customer mit gegebener ID als JSON.
        
        - getAllCustomerInfos(status)
           Liefert Liste aller Customer Infos zurück.
Pepper
----------
.. code::
  
  - RPC-Server
        - updateObserverClient(clientID, host, port)
            Der RPC-Server verwaltet eine Map von Clients und deren IPs/Ports. Bekommt er diese Anfrage updatet er die Infos des entsprechenden Clients oder legt ihn neu an.
            
        - notify()
            Benachrichtigung, dass der Kuchen geschnitten ist.
            
        - new_notify(json-string)
            Benrachrichtigung wenn eine customer order fertig ist.
