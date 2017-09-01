Kommunikation zwischen Pepper und Knowledge
==========================================================
Setup
-----
Die Implementierung der Kommunikation beschränkt sich auf die Pakete ''pepper-communication'' und ''planning-common''. Achte darauf, dass folgende sachen laufen:

- RPC Server von Franklin
- roslaunch catering_queries prolog.launch (von knowledge)

Prüfe, ob die IP in der Signatur von ''pcomm:setup-pepper-communication'' mit der IP von Franklins Server übereinstimmt. Starte den Planning Kontext durch ''(pexec::init-planning :use-pepper t)'', der Switch use-pepper sorgt für die initialisierung der RPC Schnittstelle. 

Informationen in Knowrob aktualisieren
--------------------------------------
Für die Ablage von Wissen aus dem Dialog werden JSON Strings von Pepper, über Planning per Prolog zu Knowrob gesendet. Pepper ruft dafür die RPC Funktion ''assertDialogElement'' von Plannings RPC Server auf, die einen JSON String als Parameter erwartet::  
     
     (defun |assertDialogElement| (json-string)
          "Calls `handle-knowledge-update' to handle the json object containing information from the 
          dialog for the knowledgebase. Returns a JSON string with the response."
          (handle-knowledge-update json-string))

Wie die JSON Objekte auszusehen haben ist in diesem Doc festgehalten: https://docs.google.com/document/d/1wCUxW6c1LhdxML294Lvj3MJEqbX7I0oGpTdR5ZNIo_w

Die Funktion ''handle-knowledge-update'' filtert Quotes und Spaces aus dem JSON-String und schickt ihn an Knowrob. Danach brauchen wir eine response für Pepper. Wir konstruieren ein JSON mit der CustomerId, dem query typ und dem value ''success'' auf 1. Falls es sich bei dem JSON von Pepper um eine query vom typ ''setCake'' handelt, schicken wir zusätzlich den nächstbesten freien Platz zurück. Dieser freie Platz wird dem Gast zugewiesen, indem ein JSON mit query typ ''setLocation'' an Knowrob gesendet wird.

Informationen aus der Knowledgebase abrufen
-------------------------------------------
Die RPC Funktionen ''getCustomerInfo'' und ''getAllCustomerInfos'' liefern Informationen zu Bestellungen. Die entsprechenden Funktionen ''thread-get-customer-info'' und ''thread-get-all-customer-info'' besorgen sich die Informationen aus Knowrob, fürgen sie zu einer associative list zusammen und lassen diese zu einem JSON parsen. Der entstandene String geht zurück an Pepper.
