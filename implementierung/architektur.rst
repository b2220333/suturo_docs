=============
Systemarchitektur
=============
Die zugrunde liegende Systemarchitektur lässt sich in fünf Bereiche aufteilen: Die vier Gruppen *Perception*, *Manipulation*, *Knowledge* und *Planning* sowie *Pepper*.

.. figure:: architektur.png


*Pepper* stellt die Schnittstelle zum Gast dar. Sie nimmt seine Bestellung entgegen und gibt sie an *Planning* weiter. Dadurch wird bei *Planning* der Top-Level-Plan für den *PR2* angestoßen. Je nach Art der erhaltenen Anweisung werden die zugehörigen Pläne ausgeführt. Dabei kann *Planning* die Pipeline von *Perception* ausführen, um verschiedene Objekte zu erkennen oder den Status bereits bekannter Objekte bei *Knowledge* erfragen. Roboterbewegungen werden über die Schnittstelle zu *Manipulation* iniitiert. *Manipulation* horcht auf die Szenenvervollständigung von *Perception*, um Kollisionen zu vermeiden.
