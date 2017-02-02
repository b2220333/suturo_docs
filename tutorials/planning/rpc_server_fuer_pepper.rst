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

Starting up the Server!
^^^^^^^^^^^^^^^^^^^^^^^^

What is the point of sending informations to a server if it is switched off? That's right! None!
Here we will start up our XML-RPC Server in two easy steps!

1. After starting roscore and the whole shabang use

      ``$ rosrun pepper_communication rpc_server.py``
   
   A message telling you the server is ready to use should appear.

2. That's it, note the IP adress of the machine via *ifconfig* or whatnot and use that.


Call it from LISP
-----------------

Now you can call the server from any client, in this example we use the LISP xml-rpc library. First we include 's-xml-rpc' in the asdf file::

    (defsystem pepper-communication-system
      :depends-on (roslisp std_msgs-msg s-xml-rpc) 
      ...

Then we can use the **xml-rpc-call** function from the library, which needs a xml encoded message and the IP adress and port of the server. Look up the IP adress and port in the servers implementation. Then we need to encode the message. Thats where **encode-xml-rpc-call** comes in handy::

    (encode-xml-rpc-call <remote-function-name> <argument1> <argument2> ...)

We want to wrap that up into a function called fire-rpc. The default target function is **setStatus**, host and port are defined globally. So the final construct looks like that::

    (defparameter *host* "127.0.0.1")
    (defparameter *port* 8000)

    (defun fire-rpc (arguments &optional (remote-function "setStatus"))
      (s-xml-rpc:xml-rpc-call
        (s-xml-rpc:encode-xml-rpc-call remote-function arguments)
        :host *host*
        :port *port*))
