=============
Schnittstellen
=============

Im Folgenden finden sich die Definitionen der Schnittstellen der einzelnen Gruppen sowie übergreifender Module.


Perception
----------
TODO

Knowledge
----------
.. code::
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
TODO: Controller-Spezifikation bitte ins Wiki schreiben.


Planning
----------
.. code::
- RPC-Server
        - cutCake()
            Um den Plan zum Kuchen schneiden anzustoßen. Soll sofort zurückgeben, wie lange das etwa dauern wird (also z.B. wie viele Aufträge vorher noch ausgeführt werden müssen).


Pepper
----------
.. code::
- RPC-Server
        - notify()
            Benachrichtigung, dass der Kuchen geschnitten ist.
