

Installation: Start ueber Pepper
=================================

In diesem Tutorium wird zum einen beschrieben, wie Pepper- und PR2-Roboter miteinander verbunden werden. Zum anderen lernt man, wie das Sprachdialogsystem von Pepper gestartet wird.


Physikalische Verbindung Pepper-PR2
-----------------------------------

Da eine lokale(vom Roboter aus) Ausführung des Sprachdialogsystems ungeheuer starke Änderungen an dem auf dem Roboter vorliegenden Grundsystem fordern würde und erst seit ein Paar Monaten  laut Aldebarans Ingenieure möglich ist, wird eine Fernausführung(vom Ferncomputer aus) von Anwendungen empfohlen. Aus diesem Grund wird das Sprachdialogsystem von Pepper auf einem Ferncomputer(Proxy) ausgeführt, welcher über ein lokales Netzwerk(WiFi/Ethernet) sowohl mit PR2 auch mit Pepper verbunden ist.

Installation und Einstellungen von Programmen
---------------------------------------------

Betriebsystem
^^^^^^^^^^^^^

Auf dem Proxy-Computer muss ein Betriebsystem Ubuntu 14.04 LTS 64bits installiert werden::

     http://releases.ubuntu.com/14.04/


Ros-System
^^^^^^^^^^^

Auf dem Proxy-Computer muss ein Ros-Indigo-System installiert werden::

    http://wiki.ros.org/indigo/Installation/Ubuntu

