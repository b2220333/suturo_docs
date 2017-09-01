
Package object_state 
-------------------------------

**MS1:**
	Das Package stellt einen listener für das Perception-Topic 'percepteros/object_detection' bereit und hört es ab. Die Informationen aus dem Perception-Topic werden dann weiterverarbeitet, indem eine KnowRob-Repräsentation erzeugt wird. Mittels dieser können wir dann Wissen inferieren.

**MS2:**
	Das Package stellt nun weitere Funktionalitäten bereit. Es können beispielsweise die Position und Orientierung von gültigen Fluent-Objekten über ein Pythonskript auf das tf topic gepublished werden.

**MS3:**
	Python Skripte ausführbar aus Prolog Queries über Prython.
	Mehrere Objekte des gleichen Typs jetzt möglich. Der Objektname wird aus dem Objekttypen durch anhängen eines Integerwertes erzeugt.

**MS4:** 
	Es wurden einige Erweiterungen hinzugefügt, um mit den während des 3. Meilensteins eingeführten multiplen Objekten konsistent zu bleiben. Prython wurde in diesem Meilenstein weiterentwickelt und wird bereits in den neuen Funktionen verwendet.

**MS5:**
	Erweiterung um generische Methoden zur Ablage und Abfrage von jedweder Information und Wissen in die KB.

Das Package enthält:

* Prolog-Module

  * ('prolog_object_state.pl'): wird für alle wichtigen Features in Knowledge verwendet.
  * ('prython.pl'): enthält die Funktionalität in Prolog Python-Funktionen und Objekte zu nutzen.

* Skripts
  
  * ('subscriber.py'): hört das ROS-Topic 'percepteros/object_detection' ab und verarbeitet die gesammelten Daten für die Repräsentation in KnowRob weiter.
  * ('fluents_tf_publisher.py'): ein Publisher, der bei Bedarf Objektposition, -Orientierung und -Dimensionen an TF schicken kann.
  * ('dummy_publisher.py'): simuliert Perzeption, die vom Subscriber verarbeitet werden; eher für Testzwecke relevant.

* Launchfiles

  * ('object_state.launch'): startet alle Module von Knowledge nacheinander.
  * ('prolog.launch'): startet nur das Prolog-Modul; eher für manuelles Testen relevant.


Verwendung und Testlauf von objectDetection
--------------------------------------------

.. note:: Öffnet eine Kommandozeile à la Terminator und teilt sie in 4 Shells auf, da es sonst extrem unübersichtlich wird. Im weiteren sind die unterschiedlichen Terminals mit t1(oben links), t2(oben rechts), t3(unten links) und t4(unten rechts) benannt.

1. Schritt - Launch Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Wir starten zunächst die Prolog Komponente in t1::

	roslaunch object_state prolog.launch

.. note:: roslaunch startet, sofern noch nicht vorher geschehen, einen roscore. Ihr braucht also keinen separaten roscore zu starten.

2. Schritt - Publishen von Objekten
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Wir publishen Daten auf dem Topic, welches der Subscriber abhört. Hierbei können entweder manuell die Objekte gepublished oder der dummy_publisher genutzt werden.

Dummy Publisher
"""""""""""""""

Der Dummy Publisher erzeugt in zufälliger Weise Object perceptions und posted diese. Dazu starten wir::

	rosrun object_state dummy_publisher.py

Hierbei stehen 1-6 Typen und zurzeit 1 Objekt pro Typ zur Verfügung. Diese werden mit allen nötigen Daten (FrameID, Pose, etc.) erzeugt.

Manuell publishen
"""""""""""""""""

Dazu können wir folgendes Kommando in t2::

	percepteros/object_detection suturo_perception_msgs/ObjectDetection

Danach mehrfach Tab drücken um ein Aufruf-Template zu erzeugen, oder alternativ den nachfolgenden Aufruf verwenden::

	rostopic pub percepteros/object_detection suturo_perception_msgs/ObjectDetection "name: 'cylinder'
	pose:
	  header:
	    seq: 0
	    stamp: {secs: 0, nsecs: 0}
	    frame_id: 'odom_combined'
	  pose:
	    position: {x: 4.0, y: 6.0, z: 0.5}
	    orientation: {x: 3.0, y: 8.0, z: 6.0, w: 4.0}
	type: 1
	width: 7.0
	height: 5.0
	depth: 4.0" -r 15



3. Schritt - Launch Subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Jetzt können wir den Subscriber in t3 starten::

	roslaunch object_state subscriber_py.launch

Dieser erhält jetzt mit der zuvor festgelegten Frequenz von 15 Herz die Daten die auf das Topic 'percepteros/object_detection' gepublished werden. Es ist möglich, dass durch die verwendung des Launchfiles kein sichtbarer Output generiert wird, es werden trotzdem Daten empfangen.

..note:: (Optional)
Wenn unbedingt visueller Output erwünscht ist, kann der Subscriber auch ohne Launchfile gestartet werden, und zwar so::

	rosrun object_state subscriber.py

4. Schritt - Prolog Queries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Wir wollen jetzt mit einer Prolog-Query das von uns gepublishte Objekt, für das mittlerweile durch die Funktionalität des Subscribers und der in 'object_state.pl' definierten Funktionen eine KnowRob-Repräsentation erzeugt wurde, anfragen. Dazu nutzen wir den Rosservice simple_query, der es uns ermöglicht, Prolog-Anfragen zu stellen und führen das folgende Kommando in t4 aus::

	rosservice call /json_prolog/simple_query

und vervollständigen den Aufruf wie gewohnt mit doppel-Tab.

get_object_infos
""""""""""""""""

Um die Objektinformationen (Name, FrameID, Timestamp, Height, Weight, Depth) gesehener Objekte zu bekommen, verwenden den folgenden Aufruf::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '1337'
	query: 'get_object_infos(Name,FrameID,Timestamp,Height,Width,Depth)'"

.. note:: Sollen nur bestimmte Objektinfos gesucht werden, können die Variabeln des Queries durch Werte ersetzt werden, z.B.:
	Name = knowrob:cylinder
	Frame = "odom_combined"
	Timestamp = Float (Sekunden seit 01-01-1970 ~ 1.486E9)
	H, W, D = Float

Wir wollen nun die Funktion 'get_object_infos()' aufrufen, um alle möglichen Informationen über ein Objekt, von dem wir nur den Namen wissen, zu bekommen.
Zum Testen könnt ihr eine beliebige ID verwenden, müsst aber darauf achten, dass ihr im nächsten Schritt die selbe ID wieder angebt.

Es wird noch ein weiterer Prolog-Aufruf benötigt um unsere Antwort zu generieren (hier unbedingt die selbe ID wie zuvor verwenden!)::

	rosservice call /json_prolog/next_solution "id: '1337'" 

Wie erwartet liefert uns die Methode alle aktuellen Werte des Objekts zurück::

	rosservice call /json_prolog/next_solution "id: '1337'"
	status: 3
	solution:
	{"W":2,	"Name":"http://knowrob.org/kb/knowrob.owl#cone",
	"FrameID":"odom_combined","H":1,"Time":1.4867326416517348E9,"D":5}


Aber was, wenn sich nun die Werte unseres Objekts verändern?
Kein Problem für object_state!

Wir ändern einfach Mal einen Wert (Depth von 4.0 auf 88.0), und publishen das veränderte Objekt wie zu Beginn via Kommando in t2::

	rostopic pub percepteros/object_detection suturo_perception_msgs/ObjectDetection "name: 'cylinder'
	pose:
	  header:
	    seq: 0
	    stamp: {secs: 0, nsecs: 0}
	    frame_id: 'odom_combined'
	  pose:
	    position: {x: 4.0, y: 6.0, z: 0.5}
	    orientation: {x: 3.0, y: 8.0, z: 6.0, w: 4.0}
	type: 1
	width: 7.0
	height: 5.0
	depth: 88.0" -r 15

Jetzt Fragen wir erneut mittels Query das Objekt in t4 an(andere ID, da neuer Prolog Aufruf!)::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '133788'
	query: 'get_object_infos(knowrob:cylinder,Frame,Height,Width,Depth)'" 
	ok: True
	message: ''

Nun brauchen wir, wie zuvor auch schon, den zweiten Prolog-Call um die Lösung der Anfrage zu generieren (hier wieder die selbe ID nutzen, wie im Schritt zuvor!)::
	
	rosservice call /json_prolog/next_solution "id: '133788'" 

É voila, die Werte des Objekts haben sich auf magische Weise verändert::
	
	status: 3
	solution: {"Height":5,"Depth":88,"Frame":"odom_combined","Width":7}

seen_since
""""""""""

Die Query seen_since(Name, FrameID, Timestamp) soll die Frage beantworten, ob ein bestimmtes Objekt seit Zeitpunkt t1 gesehen wurde. Hierzu wird die Funktion ähnlich wie get_object_infos aufgerufen. Hierbei muss mindestens die Variable Timestamp zugewiesen sein! ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '1'
	query: 'seen_since(knowrob:cone,"odom_combined",1.486E9)'"	

Als Antwort auf eine Query mit allen Variablen festgelegt, bekommen wir folgende Antworten auf next_solution für True::

	status: 3
	solution: {}
	
oder False::
	
	status: 3
	solution: 



Verwendung und Test des TF Broadcasters
----------------------------------------

Für die Erweiterung des Packages 'object_state' wurde die Prolog-Klasse 'prolog_object_state.pl' erweitert. Außerdem wurde 'fluents_tf_broadcaster.py' als neues Pythonskript implementiert.
Ziel dieser Erweiterungen war, die Position und Orientierung aus offenen Fluent-Objekten an das tf-topic zu publishen.

Umsetzung, Verwendung und Test der neuen Funktionalität wird hier Schritt für Schritt dokumentiert.

Zunächst öffnen wir auf einem freien Workspace vier Shells. Dabei stehen im Folgenden die Abkürzungen T1-T4 für die vier Shells, wobei die Zuordnung wie folgt aussieht: T1 oben links, T2 oben rechts, T3 unten links, T4 unten rechts.

1. Schritt - Prolog laden
^^^^^^^^^^^^^^^^^^^^^^^^^

Wir beginnen damit, das Prolog-Modul des 'object_state'-Packages zu starten (ROS-Core wird automatisch mitgestartet).::

	roslaunch object_state object_state_prolog.launch

2. Schritt - Queries
^^^^^^^^^^^^^^^^^^^^
	
Als nächstes nutzen wir den Service /json_prolog/simple_query um mittels der in Prolog implementierten Dummy-Methoden echte Objektwahrnehmungen zu simulieren. Dazu kopieren wir das Folgende Kommando in T2 und lösen mittels doppeltem Drücken der TAB-Taste die automatische Vervollständigung aus.
(Als Parameter übergeben wir irgeneinen Namen, z.B. "`baum"' sowie eine beliebige ID): ::
	
	rosservice call /json_prolog/simple_query

Der vollständige Aufruf sieht dann etwa so aus: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '1'
	query: 'dummy_perception(baum)'" 

Dieses Kommando ruft die Prolog-Funktion dummy_perception(Name) auf, welche die die KnowRob-interne Repräsentation für Objekte erzeugt.

Jetzt kopieren wir (wieder per doppel TAB vervollständigen) in T2::

	rosservice call /json_prolog/next_solution

In dem erzeugten Aufruftemplate setzen wir die Id auf den selben Wert wie im vorherigen Kommando. Durch den Aufruf von next_solution wird die zuvor gestellte Query ausgeführt und wir erhalten eine Lösung, wenn es eine gibt.

Da wir die Query sauber schließen wollen, um die verwendete ID wieder verwendbar zu machen führen wir noch folgendes in T2 aus::

	rosservice call /json_prolog/finish "id: '1'"

3. Schritt - TF-Broadcaster starten
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Wir starten jetzt in T3 den TF-Broadcaster, indem wir mit folgendem Kommando das Pythonskript fluent_tf_publisher.py ausführen.::

	rosrun object_state fluents_tf_publisher.py

Auf der Konsole sollte sofort ersichtlich sein, dass der Publisher anfängt zu arbeiten. Die Textausgabe dient nur zur Information und wird vermutlich noch häufiger angepasst.

4. Schritt - Testen
^^^^^^^^^^^^^^^^^^^

Jetzt wollen wir überprüfen, was auf dem TF-Topic ankommt, dazu wechseln wir zu T4 und führen folgendes Kommando aus, um wiederzugeben, was im TF-Topic gepublished wird.::

	rostopic echo /tf

Wir sehen jetzt, dass mit hoher Frequenz die von der Dummy-Funktion erzeugten Werte gepublished werden. Soweit sogut. Wir verwenden jetzt zwei weitere Dummy-Funktionen, um zu überprüfen, wie sich die gepublishten Werte aktualisieren, wenn die Fluents "`updaten"'. Dazu führen wir in T2 folgendes aus::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '2'
	query: 'dummy_perception_with_close(baum)'" 

Meistens ist es einfacher, den beginn des Kommandos in die Konsole zu schreiben und mittels doppel-TAB das Kommando vervollständigen zu lassen. Die Werte könnt ihr dann so setzen wie oben zu sehen ist.

Danach führen wir wieder:: 

	rosservice call /json_prolog/next_solution "id: '2'" 

aus, um die Lösung zu generieren. In T4 können wir nun live beobachten, wie sich die Werte verändern. Somit haben wir erfolgreich die Veränderung von Fluent-Werten (Position, Orientierung) an das TF-Topic übertragen. Hier können sie jetzt für viele andere Aufgaben ausgelesen und weiterverwendet werden.

Der Vollständigkeit halber, sollte nun noch das Query in T2 geschlossen werden.::

	rosservice call /json_prolog/finish "id: '2'" 



Verwendung und Test des Features zum Frames verbinden
-----------------------------------------------------

.. note:: Es empfiehlt sich ein Terminal-Tool wie Terminator zu verwenden, dass mehrere Shells in einem Fenster (split view) anzeigen kann.

Erzeugt fünf Shells und ordnet sie ca gleich groß nebeneinander an. Im weiteren sind die unterschiedlichen Terminals mit t1(oben links), t2(oben mitte), t3(oben rechts), t4(unten links) und t5(unten rechts) benannt.


1. Schritt - Prolog laden
^^^^^^^^^^^^^^^^^^^^^^^^^
In t1 ausführen: ::

	roslaunch object_state prolog.launch

2. Schritt - Testframes erzeugen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In t2 ausführen: ::

	rosrun object_state test_frame.py

Das Pythonskript stellt zwei Frames zur Verfügung, die zum Testen benötigt werden.

3. Schritt - Testobjekt erzeugen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. note:: Achte darauf, für jeden Aufruf von simple_query eine neue ID zu verwenden. Beim Aufruf von next_solution muss zudem *immer* die ID des vorausgegangenen simple_query Aufrufs gesetzt werden.

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '1'
	query: 'connect_frames1(carrot1)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '1'"
	rosservice call /json_prolog/finish "id: '1'"


The More You Know: connect_frames1 erstellt knowrob-intern eine Objektrepräsentation als Fluents. Der Name des Objekts ist "carrot1" und die Position(xyz) ist [5,4,3]. Die restlichen Parameter sind für diesen Test nicht so wichtig.

4. Schritt - Verbinden der Frames
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In t4 ausführen: ::

	rosrun object_state connect_frames_bridge.py 

Das Skipt dient einfach nur dazu die Prolog-Funktion connect_frames() mit den richtigen Parametern aufzurufen.

In t5 kopieren, TAB-vervollständigen, parentFrame und childFrame setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /connect_frames_service 

Sollte in etwa so aussehen: ::

	rosservice call /connect_frames_service "parentFrame: '/turtle1'
	childFrame: '/carrot1'" 


5. Schritt - Überprüfen von Position und Orientierung
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Wenn die Frames korrekt verbunden wurden, sollten sich die Positions- und Orientierungswerte verändert haben.  

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '2'
	query: 'get_tf_infos(carrot1, FrameID, Position, Orientation)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '2'"
	rosservice call /json_prolog/finish "id: '2'"

An der Ausgabe können wir erkennen, dass sich die Werte für die Position und Orientierung verändert haben, von [5,4,3] zu [0,-2,0] bzw. [6,7,8,9] zu [0,0,0,1]

6. Schritt - Überprüfen des Verhaltens bzgl. der Fluent-Updates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Wir prüfen jetzt zusätzlich, ob die neue Position und Orientierung *gleich bleibt*, wenn wir ein Update dieser Werte für die Fluents in knowrob durchführen. 
Da unser Objekt nun fest mit einem Frame verbunden ist, sollten sich dessen Werte natürlich *nicht mehr* verändern!

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '3'
	query: 'connect_frames2(carrot1)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '3'"
	rosservice call /json_prolog/finish "id: '3'"

Im wesentlichen wiederholen wir jetzt die Prozedur aus Schritt 5, um zu überprüfen, dass sich die Positions- und Orientierungswerte nicht verändert haben.

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '4'
	query: 'get_tf_infos(carrot1, FrameID, Position, Orientation)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '4'"
	rosservice call /json_prolog/finish "id: '4'"

Wie du siehst, haben sich weder die Werte für Position noch Orientierung verändert!

Jetzt müssen wir aber noch sicherstellen, dass *andere* Objekte noch erstellt und über ihre zugehörigen Fluents noch wie gewohnt verändert werden können.

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '5'
	query: 'connect_frames1(baum)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '5'"
	rosservice call /json_prolog/finish "id: '5'"

Nun sollte ein neues Objekt vom Typ baum existieren.
Schauen wir auf die Werte.

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '6'
	query: 'get_tf_infos(baum, FrameID, Position, Orientation)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '6'"
	rosservice call /json_prolog/finish "id: '6'"

Die Werte für Position: [5,4,3].

Und prüfen wir letztlich noch, ob sich die Werte verändern lassen. Dies sollte funktionieren, da ja dieser Frame nicht verbunden ist!

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '7'
	query: 'connect_frames4(baum)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '7'"
	rosservice call /json_prolog/finish "id: '7'"

Werfen wir nun einen Blick auf die Werte.

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '8'
	query: 'get_tf_infos(baum, FrameID, Position, Orientation)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '8'"
	rosservice call /json_prolog/finish "id: '8'"

Die Werte haben sich aktualisiert: [8,7,7]

Damit verhält sich die Funktion connect_frames() genau so, wie wir es wollen.

7. Schritt - Frames voneinander lösen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Letztlich muss noch gewährleistet werden, dass sich Frames auch wieder ordentlich voneinander lösen lassen.

Wir führen also zunächst die disconnect_frames() Funktion aus.

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '9'
	query: 'connect_frames3(a,b)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '9'"
	rosservice call /json_prolog/finish "id: '9'"

Die Frames sollten jetzt wieder voneinander gelöst sein.
Eine letzte Query müssen wir noch ausführen, um dies zu überprüfen.

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '10'
	query: 'connect_frames2(carrot1)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '10'"
	rosservice call /json_prolog/finish "id: '10'" 

Und jetzt final schauen, ob die Werte sich wieder angepasst haben.

In t3 kopieren, TAB-vervollständigen, Id und Query setzen (vgl. weiter unten) und dann ausführen: ::

	rosservice call /json_prolog/simple_query

Sollte in etwa so aussehen: ::

	rosservice call /json_prolog/simple_query "mode: 0
	id: '11'
	query: 'get_tf_infos(carrot1, FrameID, Position, Orientation)'" 

In t3 ausführen: ::

	rosservice call /json_prolog/next_solution "id: '11'"
	rosservice call /json_prolog/finish "id: '11'" 

Die Werte haben sich wie gewünscht wieder verändert. Damit ist unser Feature funktionsfähig.
