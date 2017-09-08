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
    
    :Beschreibung: Steuert ein Positionsziel für die Greiferöffnung an.
    :Gelenklisten: 
      - *graspkard/config/pr2_right_gripper.yaml*: Rechter Greifer
      - *graspkard/config/pr2_left_gripper.yaml*: Linker Greifer
    :Parameter:
      - **double**: Angestrebte Öffnungsweite in *m*
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/grasp_l_50.yaml* Zugreifen mit dem linken Greifer bei 50% Stärke
      - *graspkard/test_params/grasp_l_50.yaml* Zugreifen mit dem rechten Greifer bei 50% Stärke
      - *graspkard/test_params/release_l_50.yaml* Loslassen mit dem linken Greifer bei 50% Stärke
      - *graspkard/test_params/release_l_50.yaml* Loslassen mit dem rechten Greifer bei 50% Stärke

graspkard/pr2_grasp_control_r.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Fährt eine Greifpose an einem Zylinder mit dem rechten Arm an. Öffnet hierbei den Greifer.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, Rechter Arm und Greifer
    :Parameter:
      - **transform**: Frame des Zylinders im Referenzframe des Roboters. Beim PR2 ist dies *base_link*.
      - **double**: Durchmesser des Zylinders in *m*
      - **double**: Höhe des Zylinders in *m*
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/approach_cylinder_r.yaml*: Fährt mit dem rechten Arm einen Zylinder namens *cylinder* an, welcher die Maße 5x14 *cm* hat. 

graspkard/pr2_grasp_control_l.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Fährt eine Greifpose an einem Zylinder mit dem linken Arm an. Öffnet hierbei den Greifer.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_left_arm.yaml*: Torso, Linker Arm und Greifer
    :Parameter:
      - **transform**: Frame des Zylinders im Referenzframe des Roboters. Beim PR2 ist dies *base_link*.
      - **double**: Durchmesser des Zylinders in *m*
      - **double**: Höhe des Zylinders in *m*
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/approach_cylinder_l.yaml*: Fährt mit dem linken Arm einen Zylinder namens *cylinder* an, welcher die Maße 5x14 *cm* hat.
        
graspkard/pr2_upper_body_joint_control.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Fährt ein Gelenkziel für den Oberkörper des Roboters an. 
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body.yaml*: Torso, beide Arme, keine Greifer
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
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/upper_body_praying_mantis.yaml*: Die *Gottesanbeterin*-Pose

graspkard/pr2_right_arm_joint_control.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Fährt ein Gelenkziel für den rechten Arm des Roboters an. Der Greifer ist zwar Teil der Gelenkliste und Parameter, wird aber nicht kontrolliert.
    :Gelenklisten: 
      - *graspkard/config/pr2_right_arm.yaml*: Rechter Arm und Greifer
    :Parameter:
        - **double** Position des Gelenks *r_shoulder_pan_joint* in *rad*
        - **double** Position des Gelenks *r_shoulder_lift_joint* in *rad*
        - **double** Position des Gelenks *r_upper_arm_roll_joint* in *rad*
        - **double** Position des Gelenks *r_elbow_flex_joint* in *rad*
        - **double** Position des Gelenks *r_forearm_roll_joint* in *rad*
        - **double** Position des Gelenks *r_wrist_flex_joint* in *rad*
        - **double** Position des Gelenks *r_wrist_roll_joint* in *rad*
        - **double** Position des Greifers in *m* - wird ignoriert
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/r_arm_praying_mantis.yaml*: Die *Gottesanbeterin*-Pose des rechten Arms

graspkard/pr2_left_arm_joint_control.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Fährt ein Gelenkziel für den linken Arm des Roboters an. Der Greifer ist zwar Teil der Gelenkliste und Parameter, wird aber nicht kontrolliert.
    :Gelenklisten: 
      - *graspkard/config/pr2_left_arm.yaml*: Rechter Arm und Greifer
    :Parameter:
        - **double** Position des Gelenks *l_shoulder_pan_joint* in *rad*
        - **double** Position des Gelenks *l_shoulder_lift_joint* in *rad*
        - **double** Position des Gelenks *l_upper_arm_roll_joint* in *rad*
        - **double** Position des Gelenks *l_elbow_flex_joint* in *rad*
        - **double** Position des Gelenks *l_forearm_roll_joint* in *rad*
        - **double** Position des Gelenks *l_wrist_flex_joint* in *rad*
        - **double** Position des Gelenks *l_wrist_roll_joint* in *rad*
        - **double** Position des Greifers in *m* - wird ignoriert
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/l_arm_praying_mantis.yaml*: Die *Gottesanbeterin*-Pose des linken Arms

graspkard/pr2_place_control_r.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Platziert ein mit rechts gegriffenes, zylindrisches Objekt in einer Zielzone.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, Rechter Arm und Greifer
    :Parameter:
      - **transform** Frame der Zielzone in *base_link*.
      - **transform** Frame des Zylinders in *r_wrist_roll_link*.
      - **double** Durchmesser des Zylinders
      - **double** Höhe des Zylinders
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/place_cylinder_r.yaml*: Platziert einen Zylinder namens *cylinder* in einem Zielareal namens *goal_area*

graspkard/pr2_place_control_l.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Platziert ein mit links gegriffenes, zylindrisches Objekt in einer Zielzone.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_left_arm.yaml*: Torso, Rechter Arm und Greifer
    :Parameter:
      - **transform** Frame der Zielzone in *base_link*.
      - **transform** Frame des Zylinders in *l_wrist_roll_link*.
      - **double** Durchmesser des Zylinders
      - **double** Höhe des Zylinders
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/place_cylinder_l.yaml*: Platziert einen Zylinder namens *cylinder* in einem Zielareal namens *goal_area*


graspkard/knife_grasp.yaml - Messer greifen
"""""""""""""""""""""""""
    :Beschreibung: Fährt ein Messer zum Greifen mit dem rechten Arm an.
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, rechter Arm, rechter Greifer
    :Parameter:
      - **transform** Frame des Messers in *base_link*
      - **double** Höhe des Messers in *m*
      - **double** Länge des Messergriffes in *m*
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter: TODO


graspkard/TODO - Messer umgreifen
"""""""""""""""""""""""""
    :Beschreibung: Messer sitzt beim ersten Greifen ungeeignet für das Schneiden im Greifer und wird mit Hilfe dieses Controllers in eine geeignete Position gebracht.
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_grippers.yaml*: Torso, rechter Arm, linker Arm, rechter Greifer, linker Greifer
    :Parameter:
      - **transform** Frame des Messers in *base_link*
      - **double** Länge des Messers in *m*
      - **double** Länge des Griffes in *m*
      - **double** Höhe des Griffes in *m*
    :Feedback: *feedback* je näher an :math:'0', desto besser 
    :Beispiel-Parameter: TODO


graspkard/pr2_cut_r.yaml
"""""""""""""""""""""""""
    :Beschreibung: Schneidet einen Kuchen parallel zu seiner YZ-Ebene mit dem rechten Arm.
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, Rechter Arm und Greifer
    :Parameter:
      - **transform** Frame des Kuchens in *base_link*
      - **double** Länge des Kuchens (X-Ausdehnung)
      - **double** Breite des Kuchens (Y-Ausdehnung)
      - **double** Tiefe des Kuchens (Z-Ausdehnung)
      - **transform** Frame des Messers in *r_wrist_roll_link*
      - **double** Höhe des Messers
      - **double** Länge des Messergriffs
      - **double** Breite des Kuchenstücks
    :Feedback: *feedback* Je näher an 0 desto besser.
    :Beispiel-Parameter: *graspkard/test_params/cut.yaml*: Schneidet ein 1,5cm breites Stück von einen Kuchen *cake* mit einem Messer *knife*.


graspkard/pr2_cut_position_r.yaml
"""""""""""""""""""""""""
    :Beschreibung: Geht mit dem rechten Arm in eine Vorpose, um einen Kuchen zu schneiden.
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, Rechter Arm und Greifer
    :Parameter:
      - **transform** Frame des Kuchens in *base_link*
      - **double** Länge des Kuchens (X-Ausdehnung)
      - **double** Breite des Kuchens (Y-Ausdehnung)
      - **double** Tiefe des Kuchens (Z-Ausdehnung)
      - **transform** Frame des Messers in *r_wrist_roll_link*
      - **double** Höhe des Messers
      - **double** Länge des Messergriffs
      - **double** Breite des Kuchenstücks
    :Feedback: *feedback* Je näher an 0 desto besser.
    :Beispiel-Parameter: *graspkard/test_params/cut_pos.yaml*: Geht in die Vorpose um schließlich ein 1,5cm breites Stück von einen Kuchen *cake* mit einem Messer *knife*.


graspkard/pr2_detatch_knife_r.yaml
""""""""""""""""""""""""""
    
    :Beschreibung: Löst ein mit rechts gegriffenes, Objekt von einem Magnet-Rack. Die Y-Achse muss in das Rack hinein zeigen. Diese Ausrichtung wurde gewählt, da man so die letzte Pose des Messers als Pose für das Rack verwenden kann.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, Rechter Arm und Greifer
    :Parameter:
      - **transform** Frame des Messers in *r_wrist_roll_link*.
      - **transform** Frame des Racks in *base_link*.
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - Noch keine
        
graspkard/pr2_look_at.giskard
""""""""""""""""""""""""""
    
    :Beschreibung: Richtet den RGB-Sensor der Kinect auf den Mittlepunkt eines Frames aus.
    :Gelenklisten: 
      - *graspkard/config/pr2_lookAt_joints.yaml*: Torso, Neigungs- und Drehgelenk
    :Parameter:
      - **transform** Frame zum Angucken in *base_link*.
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/poi_test.yaml*
        
graspkard/pr2_grasp_plate_r.giskard
""""""""""""""""""""""""""
    
    :Beschreibung: Nutzt den rechten Arm, um eine kreisförmige Kante anzufahren. Der Mittelpunkt der Kante wird als Frame übergeben. Die Z-Achse des Frames ist die Achse um die die Kante rotiert ist. Für die Kante wird ein Neigungswinkel angegeben, der die Neigung der Kante zur Z-Achse angibt. Wiichtig bei diesem Winkel ist, dass er vom äußeren Rand zur Achse hin gemessen wird, also üblicherweise größer als 90° ist.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, rechter Arm und Greifer
    :Parameter:
      - **transform** Frame als Mittelpunkt für Kante in *base_link*.
      - **double** Radius der Kante in *m*.
      - **double** Obere Z-Koordinate der Kante im Mittelpunkts-Frame.
      - **double** Breite der Kante in *m*.
      - **double** Neigung der Kante relativ zur Z-Achse des Mittelpunkts-Frames in *rad*.
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - *graspkard/test_params/pr2_grasp_plate_r.yaml*

graspkard/pr2_release_r.giskard
""""""""""""""""""""""""""
    
    :Beschreibung: Lässt ein mit dem rechten Greifer gehaltenes Objekt los. Hierfür wird der Greifer ca 12cm entlang seiner X-Achse rückwärts bewegt, während die Rotation beibehalten wird. Damit dies funktioniert, muss dem Regler die initiale Transformation des Greifers als konstanter Frame übergeben werden. Zusätzlich muss noch angegeben werden, wie weit der Greifer geöffnet werden soll.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, rechter Arm und Greifer
    :Parameter:
      - **transform** Ausgangsframe des Greifers in *base_link*.
      - **double** Öffnungsweite des Greifers in *m*.
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
    :Beispiel-Parameter:
      - Derzeit keine
        
graspkard/pr2_move_and_flip_r.giskard
""""""""""""""""""""""""""
    
    :Beschreibung: Gedacht für den vorsichtigen Transport von etwas auf einem Teller oder Kuchenheber (o.ä) und anschließendes Ablegen (Abkippen) in einer kreisförmigen Zielzone. Benötigt wird die Transformation des Kuchenhebers relativ zum rechten Greifer, der Frame der Zielzone, die Breite des Hebers und der Radius der Zielzone.
    :Gelenklisten: 
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, rechter Arm und Greifer
    :Parameter:
      - **transform** Werkzeug relativ zu *r_wrist_roll_link*.
      - **transform** Zielzone in *base_link*.
      - **double** Breite des Werkzeugs in *m*.
      - **double** Radius der Zielzone in *m*.
    :Feedback: *feedback* je näher an :math:`0`, desto besser.
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
