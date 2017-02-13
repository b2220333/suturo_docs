=============
Schnittstellen
=============

Im Folgenden finden sich die Definitionen der Schnittstellen der einzelnen Gruppen sowie übergreifender Module.


Perception
----------
.. code::
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

.. code::
Prolog
______

- get_object_info(+Name, -FrameID, -Timestamp, -Height, -Width, -Depth) -> Liste von Lösungen
        In welcher Form der Timestamp kommt, ist für die Schnittstelle relativ unwichtig, da wir ihn nur umherreichen und es auf jeden Fall ein String sein wird.

- seen_since(+Name, +FrameID, +Timestamp) -> True/False
        Wurde das Objekt vom Typ (Name) "Name" und der Frame-ID "FrameID" seit dem Timestamp "Timestamp" wieder gesehen?

- disconnect_frames(+ParentFrameID, +ChildFrameID)
        Trennt zwei Objekte mit den gegebenen Frames, so dass die zuvor konstante Transformation genutzt wird, um die neue "absolute" Position des Objektes zu berechnen und zu publishen.
        
Service
______    
        
- connect_frames_service(String ParentFrameID, String ChildFrameID)
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
        - **double** Position des Gelenks *r_shoulder_pan_joint* in *rad*
        - **double** Position des Gelenks *r_shoulder_lift_joint* in *rad*
        - **double** Position des Gelenks *r_upper_arm_roll_joint* in *rad*
        - **double** Position des Gelenks *r_elbow_flex_joint* in *rad*
        - **double** Position des Gelenks *r_forearm_roll_joint* in *rad*
        - **double** Position des Gelenks *r_wrist_flex_joint* in *rad*
        - **double** Position des Gelenks *r_wrist_roll_joint* in *rad*
        - **double** Position des Gelenks *l_shoulder_pan_joint* in *rad*
        - **double** Position des Gelenks *l_shoulder_lift_joint* in *rad*
        - **double** Position des Gelenks *l_upper_arm_roll_joint* in *rad*
        - **double** Position des Gelenks *l_elbow_flex_joint* in *rad*
        - **double** Position des Gelenks *l_forearm_roll_joint* in *rad*
        - **double** Position des Gelenks *l_wrist_flex_joint* in *rad*
        - **double** Position des Gelenks *l_wrist_roll_joint* in *rad*
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


graspkard/TODO - Messer greifen
"""""""""""""""""""""""""
    :Beschreibung: Fährt ein Messer zum Greifen mit dem rechten Arm an.
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, rechter Arm, rechter Greifer
    :Parameter:
      - **transform** Frame des Messers in *base_link*
      - **double** Länge des Messers in *m*
      - **double** Länge des Griffes in *m*
      - **double** Höhe des Griffes in *m*
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


graspkard/TODO - Kuchen schneiden
"""""""""""""""""""""""""
    :Beschreibung: Schneidet einen Kuchen entlang einer Schnittebene mit dem rechten Arm.
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, Rechter Arm und Greifer
    :Parameter:
      - **transform** Frame des Kuchens in *base_link*
      - **transform** Frame des Messers in *r_wrist_roll_link*
      - **TODO**
    :Feedback: *feedback* **TODO**
    :Beispiel-Parameter: TODO


graspkard/TODO - Kuchenstück beiseite scheiben
"""""""""""""""""""""""""
    :Beschreibung: Schiebt ein Kuchenstück ein paar Zentimeter beiseite.
    :Gelenklisten:
      - *graspkard/config/pr2_upper_body_right_arm.yaml*: Torso, Rechter Arm und Greifer 
    :Parameter:
      - **TODO** Kuchen Frame, Kuchenhöhe, KuchenLänge, Kuchenbreite, Kuchenstück Frame, Kuchenstückhöhe, Kuchenstücklänge, Kuchenstückbreite
    :Feedback: *feedback* **TODO**
    :Beispiel-Parameter: TODO


Planning
----------
.. code::
- RPC-Server
        - updateObserverClient(clientID, host, port)
            Der RPC-Server verwaltet eine Map von Clients und deren IPs/Ports. Bekommt er diese Anfrage updatet er die Infos des entsprechenden Clients oder legt ihn neu an.
        
        - cutCake()
            Um den Plan zum Kuchen schneiden anzustoßen. Soll sofort zurückgeben, wie lange das etwa dauern wird (also z.B. wie viele Aufträge vorher noch ausgeführt werden müssen). Return -1 bei serverseitigem Fehler.
            
        - stressLevel()
            Gibt die Auslastung des Servers als numerischen Wert zurück. Entspricht der Anzahl der Aufgaben, die noch durchzuführen sind.
            
        - nextTask()
           Liefert die Beschreibung der nächsten, geplanten Aufgabe zurück.


Pepper
----------
.. code::
- RPC-Server
        - updateObserverClient(clientID, host, port)
            Der RPC-Server verwaltet eine Map von Clients und deren IPs/Ports. Bekommt er diese Anfrage updatet er die Infos des entsprechenden Clients oder legt ihn neu an.
            
        - notify()
            Benachrichtigung, dass der Kuchen geschnitten ist.
