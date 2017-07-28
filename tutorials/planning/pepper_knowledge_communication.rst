Kommunikation zwischen Pepper und Knowledge
==========================================================

Für die Ablage von Wissen aus dem Dialog werden JSON Strings von Pepper, über Planning per Prolog zu Knowrob gesendet. Pepper ruft dafür die RPC Funktion ''assertDialogElement'' von Plannings RPC Server auf, die einen JSON String als Parameter erwartet::  
     
     (defun |assertDialogElement| (json-string)
          "Calls `handle-knowledge-update' to handle the json object containing information from the 
          dialog for the knowledgebase. Returns a JSON string with the response."
          (handle-knowledge-update json-string))
