Nutzung von REST zur Kommunikation mit Pepper
==========================================================

Hier zeigen wir, wie man remote-procedure-calls (RPC) zur Kommunikation mit Pepper absetzt und empfängt. Siehe dazu die Schnittstellenbeschreibung  `hier. <https://github.com/suturo16/suturo_docs/blob/master/implementierung/schnittstellen.rst>`_ 

Setup
-----

Um Kommunikation stattfinden zu lassen ist auf allen verwendeten Robotern je ein RPC-Server, als auch Client implementiert. Wir starten mit dem Python Server und Client auf Pepper, und gehen dann zum Lisp Code für PR2 und Turtles über. Man braucht dafür Python und Roslisp.

Schreibe einen minimalen Server in Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Du brauchst die IP-Adresse deines Rechners im Netzwerk. 

.. Note: Your code can be run in a myriad of ways, I used Choreograph to pass my scripts to Pepper.

1. Schreibe ein Python Script, sodass es für Pepper kompatibel ist. Schaue dir notfalls ein paar Tutorials dazu an.

2. Das Script muss xmlrpclib importieren.

3. Definiere den Server mit deiner IP-Adresse und sinnvollem Port (ohne Leerzeichen)::
     ``server = xmlrpclib.Server('http:// >>IP ADDRESS FROM SERVER<< :7080/')``
     
Alle Calls auf den Server werden so aussehen ::
     ``server.COMMAND(PARAMETERS)``
     
Hier ist das gesamte Skript mit beispielhafter Server-Funktion, die einen String zurückgibt::

     #!/usr/bin/env python
     
     import xmlrpclib
     
     """
     Saving the server for later use.
     """
     server = xmlrpclib.Server('http://123.456.789.10:7080/')
     
     
     """
     The main class. This will run as soon as the script is used, on pepper you
     want to pack the server commands on the onStart method, not here.
     """
     if __name__ == '__main__':          
          server.echo("At the end of the test, there will be cake.")
          pass
          
Der richtige RPC-Server auf Pepper kann noch viel mehr. Er wird umfangreicher aufgesetzt, um auch ROS-Funktionen verwenden zu können. ::
     
     class Server:
          def __init__(self):
               rospy.init_node('server')
               self.confirmation="0"
               rospy.on_shutdown(self.cleanup)
               rospy.loginfo("Starting server node...")
               #read the parameter
               self.IP = rospy.get_param("~SERVER_IP", "127.0.0.1")
               self.PORT = rospy.get_param("~SERVER_PORT", "8000") 
               #Publisher
               self.pub=rospy.Publisher('~status',String,queue_size=1000)
               #rpc server
               rospy.loginfo("Starting rpc server ...")
               self.server = SimpleXMLRPCServer((self.IP, int(self.PORT)))
               self.server.register_function(self.setStatus)

Und er verwendet Funktionen wie::    
     
     def setStatus(self,status):
       #publish status of PR2
       self.pub.publish(String(status))
       return self.confirmation
       
um vom PR2 dessen Status empfangen zu können. Der aktuelle RPC-Server von Pepper liegt in https://github.com/suturo16/perception/blob/feature/pepper-robot-dialog-system/dialogsystem/nodes/rpc_server.py , aber wird vermutlich bald auf den Master Branch gemerged.

Server starten
^^^^^^^^^^^^^^
Um den Server zu starten brauchen wir den roscore. Um den aktuellen Pepper Server zu starten muss man auch den Branch des Dialogsystems im Perception Repo switchen. Achte darauf, dem Server im Python Script die richtige IP-Adresse zu geben. Im Branch existiert ein launchfile::

      ``$ roslaunch dialogsystem dialog.launch``

Alternativ startet man einfach das Script. Auf beiden Wegen steht der Server und kann angesprochen werden.

Lisp REST API
-------------

Wir brauchen die *s-xml-rpc* Bibliothek in unserem Projekt. Wir verwenden es im Paker *pepper_communication*::

    (defsystem pepper-communication-system
      :depends-on (roslisp std_msgs-msg s-xml-rpc) 
      ...

Server aufsetzen
^^^^^^^^^^^^^^^^

Für den Server muss im Emacs ein Ros-Node laufen, darin ist bereits ein RPC-Server enthalten. Wir stellen bei der Initialisierung des Server sicher, dass ein Rosnode läuft. Wenn man den Node unbedingt neu starten will, kann man der Init-Methode einen *T* Parameter übergeben.::

     (defun init-rpc-server (&optional (restart-rosnode nil))
          "Starts and initializes the RPC server and rosnode, if needed or wanted."
          (when (or (eq (roslisp:node-status) :SHUTDOWN) restart-rosnode)
               (roslisp:start-ros-node "planning")))

Nun müssen wir an IP-Adresse und Port kommen. Die Adresse kann man über die Konsole mit dem Befehl *ifconfig* herausfinden. Der Port allerdings wird dem Server vom roscore automatisch zugewiesen und ändert sich bei jedem Neustart des Nodes. Dafür gibt es aber ein kleines Codesnippet:: 

     (defun get-local-port ()
          "Returns the local port of the server."
          (nth-value 1
                     (sb-bsd-sockets:socket-name
                     (second (first s-xml-rpc::*server-processes*)))))

Diese Funktion sucht in den RPC-Prozessen nach dem Port des Servers und liefert ihn als Zahl zurück. Die Funktionen des Servers unterscheiden sich zu gewöhnlichen nur im Namen. In Lisp sind Funktionsnamen standardmäßig Symbols, d.h. uppercase Begriffe. Um sie case-sensitive zu benutzen schreibt man sie zwischen Pipes, also::

     (defun |stressLevel| ()
          "Returns the current stress level, represented by the length of tasks."
          (length *commands-list*))
          
Um die Funktionen an den Server zu Binden müssen sie in das Paket *:s-xml-rpc-exports* importiert werden. In *:s-xml-rpc-exports* liegen bei Start schon default Funktionen wie *system.listMethods*, welche die verfügbaren Serverfunktionen als Liste zurück gibt. In der Init Funktion werden alle Funktionen, die wir zur Verfügung stellen wollen, importiert::

     (defun init-rpc-server (&optional (restart-rosnode nil))
          "Starts and initializes the RPC server and rosnode, if needed or wanted."
          (when (or (eq (roslisp:node-status) :SHUTDOWN) restart-rosnode)
               (roslisp:start-ros-node "planning"))
          (import '(|sleepSomeTime|
                    |cutCake|
                    |stressLevel|
                    |nextTask|
                    |updateObserverClient|)
                  :s-xml-rpc-exports))

RPC-Client
^^^^^^^^^^
Auch die Funktionen für den Client sind in *s-xml-rpc* enthalten. Nachrichten an XML-RPC-Server müsen in XML Format verpackt werden. Dfür benutzen wir::

    (encode-xml-rpc-call <funktionsName> <argument1> <argument2> ...)

Diese Nachricht können wir an einen hypothetischen Server mit Adresse 127.0.0.1 und Port 8080 so absenden::

     (xml-rpc-call
          (s-xml-rpc:encode-xml-rpc-call <funktionsName> <argument1> <argument2> ...)
          :host "127.0.0.1"
          :port 8080)

Verpacken wir das in einer Funktion kann das so aussehen::

    (defparameter *host* "127.0.0.1")
    (defparameter *port* 8000)
    
    (defun fire-rpc (remote-function host port &rest args)  
     "Calls remote function of server with given hostname and port.
     Arguments for the remote function can be added, if needed.
     If host or port is nil, default is used."
     (when host
          (setf *host* host))
     (when port
          (setf *port* port))
     (s-xml-rpc:xml-rpc-call
          (apply 's-xml-rpc:encode-xml-rpc-call remote-function args)
          :host *host*
          :port *port*))
          
Update der Observer Informationen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Damit nicht IP-Adressen und Ports mündlich ausgetauscht werden müssen speichert sich jede Komponente des Netzwerks (PR2, Pepper, Turtle) die letzten Daten der anderen. Diese lassen sich von außen aktualisieren. In Lisp existiert dafür eine Hashmap::

     (defparameter *clients*  (alexandria:alist-hash-table '((:pepper . nil) (:turtle . nil))))
     
die nach Initialisierung nur die Keys *:pepper* und *:turtle* enthält, noch ohne Werte. Um sie zu füllen, oder besser: füllen zu lassen, stellt der Server die Funktion *updateObserverClient* bereit. Diese Empfängt IP-Adresse und Port, sowie ID des gerade sprechenden Clients, und speichert diese Daten in seiner Hashmap *clients*::

     (defun |updateObserverClient| (client-id host port)
          "Update clients' information about host and port, using the client id as primary key."
          (let ((client-key
                    (case client-id
                         ((0 "0" "pepper") :pepper)
                         ((1 "1" "turtle") :turtle)
                         (otherwise nil)))
               (error-message
                    "ERROR:
                    Usage: updateConnection(host, port, client-key)
                    Valid values for client-key are:
                    0 or 'pepper' for pepper
                    1 or 'turtle' for the turtlebot"))
             (when (not client-key)
                   (return-from |updateObserverClient| error-message))
             (when (stringp port)
                   (setf port (parse-integer port)))
             (if (gethash client-key *clients*)
                   ((lambda (client)
                         (setf (client-host client) host)
                         (setf (client-port client) port))
                      (gethash client-key *clients*))
                   (setf (gethash client-key *clients*)
                         (make-client :host host :port port)))
              'SUCCESS))
              
 Zuerst wird hier die ID, *:pepper* oder *:turtle*, des Clients ermittelt. Falls die ID nicht identifiziert werden kann, wird eine Errornachricht zurückgegeben und das Update abgebrochen. Wenn das nicht der Fall ist kann es weiter gehen. Da der Port in Calls von Lisp aus als number übergeben wird, soll er auch so gespeichert werden. Daher parsed man evtl als String übertragene Ports zur number.
 
Nun wird geprüft, ob schon Informationen zu dem Client in der Liste existieren. Entsprechend werden dann die Einträge aktualisiert oder ein neuer angelegt. 
