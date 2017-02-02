Prolog Interface benutzen
======================================

Dieser kleine Artikel geht um die Nutzung der Prolog Funktionen,  bereitgestellt von der Knowledge Gruppe. 

Setup
----------

Um die Prolog Schnittstelle zu benutzen muss das Knowledge Repo geklont werden:  `knowledge repository <https://github.com/suturo16/knowledge>`_. Für die korrekte installation siehe `SWI-Prolog installation <http://suturo.readthedocs.io/en/latest/tutorials/installation.html#swi-prolog-installieren>`_ und `KnowRob installation <http://suturo.readthedocs.io/en/latest/tutorials/installation.html#knowrob-installieren>`_.

Im Lisp Projekt müssen folgende Pakete eingebunden werden. Sie sind auch in `planning dependencies <https://github.com/suturo16/planning/blob/master/dependencies.rosinstall>`_ aufgeführt, die mit wstool einfach gezogen werden können.

*  `cram_core <https://github.com/cram2/cram_core.git>`_
*  `cram_3rd_party <https://github.com/cram2/cram_3rdparty.git>`_
*  `cram_json_prolog <https://github.com/cram2/cram_json_prolog.git>`_

Prolog Interface starten
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Zum starten der Prolog Schnittstelle, siehe `diese Anleitung. <https://github.com/suturo16/suturo_docs/blob/master/tutorials/knowledge/object_state_walkthrough.rst>`_

Prolog Funktion in Lisp aufrufen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Der Prolog Calls sieht für das Beispiel zur Anfrage der Objekt-Info so aus ::

  (defun prolog-get-object-infos (name)
    (cut:lazy-car (json-prolog:prolog
        `("get_object_infos"
                   ,(format nil "~a~a" +knowrob-iri-prefix+ name)
                   ?frame ?width ?height ?depth) :lispify T :package :pr2-do)))

Die Funktion *json-prolog:prolog* (eigentlich die Funktion *prolog* aus dem Paket *json-prolog*) versucht die Prolog Funktion mit Namen *get_object_infos* aufzurufen. Dafür braucht sie zuerst einen eindeutigen Identifier des gesuchten Objekts *name*. Die Konstante *+knowrob-iri-prefix+* ist auf *http://knowrob.org/kb/knowrob.owl#* gesetzt und repräsentiert die Adresse der Wissensbasis, kombiniert mit *name* erreicht man somit das gesuchte Objekt. Desweiteren erwartet der Prolog-Call die Bezeichner der von ihm zurückgegebenen Werte *?frame, ?width, ?height* und *?depth*. Die Prolog Funktion auf Seite der Knowledge hat also folgende Signatur :: 

  get_object_infos(?Name, ?Frame, ?Height, ?Width, ?Depth)
  
Der Prolog-Call will lispseitig die Response in Lisp-freundlichem Format bekommen, deswegen *:lispify t*, außerdem muss das Paket mit *package pr2-do* explizit angegeben werden, da es per Default auf das Paket zeigt, in dem man sich in Emacs gerade befindet, nicht, wo der Code mit dem Prolog-Call liegt. Die Response von Prolog Funktionen ist per se lazy, deswegen wird mit *cut:lazy-car* die Response in eine standard Liste verpackt.   

