=============
Pläne
=============

Im Folgenden finden sich die aktuell verwendeten Pläne (Stand: 2./3. Meilenstein). Sie sollen dem PR2 ermöglichen, einen Kuchen zu schneiden.

Den Aufenthaltsort eines Objektes überprüfen
----------
Um einen Kuchen schneiden zu können, müssen sowohl der Kuchen als auch ein Messer lokalisiert werden. Dafür wird der Plan :code:`check-object-location` genutzt, der durch eine Anfrage an Knowledge und dadurch auch indirekt an Perception überprüft, ob ein bereits bekanntes Objekt sich noch an der Stelle befindet, an der es zuletzt gesehen wurde.

.. figure:: plans/check-object-location.png

Nachdem sichergestellt ist, dass der PR2 sich in seiner Grundposition befindet, wird die Pipeline von Perception ausgeführt, um zu überprüfen, ob das gesuchte Objekt wahrgenommen wird. Dabei ist angedacht, dass der PR2 zunächst seinen Kopf in die Richtung dreht, in der sich das Objekt befinden sollte. Dies ist bisher allerdings noch nicht implementiert, da sich sowohl der Kuchen als auch das Messer im Sichtbereich des Roboters in der Grundposition befinden. Sobald die Pipeline die entsprechenden ObjectDetection-Messages publisht, kann mithilfe der Methode :code:`seen\_since(obj\_info)` von Knowledge geprüft werden, ob das gesuchte Objekt erneut gesehen wurde. Trifft dies zu, liefert der Plan zurück, dass das Objekt gefunden wurde. Ansonsten wird entsprechend zurückgeliefert, dass es nicht gefunden wurde.
Letzteres geschieht ebenfalls, wenn die Objektinformation leer ist. In diesem Fall ist das gesuchte Objekt bisher nicht bekannt und der Aufenthaltsort kann nicht überprüft werden.

Ein Messer greifen
----------
Anschließend soll das Messer gegriffen werden. Hierfür wird der Plan :code:`grasp-knife` ausgeführt, der den Greifer erst einmal an den Griff des Messers bewegt. Danach wird der Greifer des Roboters geschlossen, sodass er das Messer nun festhalten kann.
Das Greifen des Messers geschieht in zwei Schritten. Es gibt die CRAM-Funktion :code:`grasp`, die vor dem Greifen mit :code:`check-object-location` überprüft, ob das Messer gerade gesehen werden kann und dann :code:`grasp-knife` ausführt. Der Plan :code:`grasp-knife` selbst tut dies nicht, sondern geht schon davon aus, dass das Messer sichtbar ist. Diese Struktur wurde gewählt, um verschiedene Greifen-Pläne in einer Funktion zusammenfassen zu können. Vor jedem Greifvorgang soll die Objektposition überprüft werden, aber jeder Objekttyp kann unterschiedliche Low-Level-Funktionen benötigen. 
Nachdem das Messer gegriffen wurde, wird der TF-Frame des Messers in Relation zu dem Arm des Roboter gesetzt, damit es nun bei der Berechnung der Bewegungen mit berücksichtigt werden kann.

Der Plan :code:`ms2-grasp-knife` existiert speziell für diesen Meilenstein, da hier, noch bevor das Messer gegriffen wird, der TF-Frame des Messers mit :code:`odom\_combined}`durch den Service :code:`connect\_frames\_service` von Knowledge verbunden wird. Dies ist nötig, da sich sonst das Frame des Messers mit der Bewegung des Torsos, und so auch der Kinect, mitbewegen würde, und das Ziel unerreichbar machen würde. Dies ist nur für diesen Meilenstein so. In Zukunft soll eine sauberere Lösung gefunden werden.

Einen Kuchen schneiden
----------
Der Plan :code:`cut-object`, welcher zum Schneiden des Kuchens genutzt wird, geht davon aus, dass das Messer bereits gegriffen ist.

.. figure:: plans/cut-object.png

Zunächst wird die Lage des Kuchens wie oben beschrieben geprüft. Danach wird der PR2 angewiesen, eine Position anzunehmen, aus der er den Kuchen schneiden kann. D.h. er platziert den Greifer mit dem Messer über dem Kuchen. Dies geschieht über den entsprechenden Regler von Manipulation. Danach wird das eigentliche Schneiden initiiert, ebenfalls über einen Regler von Manipulation.
Wenn der Kuchen nicht gefunden wird oder im Prozess ein Fehler auftritt, wird zurückgeliefert, dass der Plan nicht erfolgreich war.
