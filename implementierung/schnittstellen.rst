=============
Schnittstellen
=============

Im Folgenden finden sich die Definitionen der Schnittstellen der einzelnen Gruppen sowie übergreifender Module.


Perception
----------
.. code::
Services
________

- runPipeline(List<Objects>) -> Liste mit Objekten für die keine Pipeline gestartet werden konnte
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

- getObjectInfo(+Name, -FrameID, -Timestamp, -Height, -Width, -Depth) -> Liste von Lösungen
        In welcher Form der Timestamp kommt, ist für die Schnittstelle relativ unwichtig, da wir ihn nur umherreichen und es auf jeden Fall ein String sein wird.

- seenSince(+Name, +FrameID, +Timestamp) -> True/False
        Wurde das Objekt vom Typ (Name) "Name" und der Frame-ID "FrameID" seit dem Timestamp "Timestamp" wieder gesehen?

- connectFrames(+ParentFrameID, +ChildFrameID)
        Verbindet zwei Objekte mit den gegebenen Frames, so dass in TF eine konstante Transformation vom Parent zum Child gepublisht wird.

- disconnectFrames(+ParentFrameID, +ChildFrameID)
        Trennt zwei Objekte mit den gegebenen Frames, so dass die zuvor konstante Transformation genutzt wird, um die neue "absolute" Position des Objektes zu berechnen und zu publishen.

Manipulation
----------
.. code::
- Controller Liste
        - Startposition
            Parameter: keine
        - Messer greifen
            Parameter: Messer Frame, Messerlänge, Grifflänge, Griffhöhe, Griffbreite
        - Messer Umgreifen
            Parameter: Messer Frame, Messerlänge, Grifflänge, Griffhöhe, Griffbreite
        - Kuchen schneiden
            Parameter: Kuchen Frame, Kuchenhöhe, KuchenLänge, Kuchenbreite
        - Kuchenstück zur Seite schieben
            Parameter: Kuchen Frame, Kuchenhöhe, KuchenLänge, Kuchenbreite, Kuchenstück Frame, Kuchenstückhöhe, Kuchenstücklänge, Kuchenstückbreite


Planning
----------
.. code::
- RPC-Server
        - updateObserverClient(clientID, host, port)
            Der RPC-Server verwaltet eine Map von Clients und deren IPs/Ports. Bekommt er diese Anfrage updatet er die Infos des entsprechenden Clients oder legt ihn neu an.
        
        - cutCake()
            Um den Plan zum Kuchen schneiden anzustoßen. Soll sofort zurückgeben, wie lange das etwa dauern wird (also z.B. wie viele Aufträge vorher noch ausgeführt werden müssen).


Pepper
----------
.. code::
- RPC-Server
        - updateObserverClient(clientID, host, port)
            Der RPC-Server verwaltet eine Map von Clients und deren IPs/Ports. Bekommt er diese Anfrage updatet er die Infos des entsprechenden Clients oder legt ihn neu an.
            
        - notify()
            Benachrichtigung, dass der Kuchen geschnitten ist.
