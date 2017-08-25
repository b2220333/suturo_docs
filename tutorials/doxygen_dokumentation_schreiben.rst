Tutorial: Doxygen Dokumentation schreiben
================================

Kommentare im Code machen
--------------

Für Doxygen sollte der Code über entsprechende Kommentare verfügen, sodass bei Methoden klar ist was sie machen. Dafür gibt es entsprechende Syntax, die man sich unter `c++ <http://csweb.cs.wfu.edu/~fulp/CSC112/codeStyle.html>`_ für C++ anschauen kann.

Das Schreiben der entsprechenden Kommentare wird durch Texteditorplugins erleichtert. Für Atom gibt es dafür `docblockr <https://atom.io/packages/docblockr>`_, für sublime `DoxyDoxygen <https://packagecontrol.io/packages/DoxyDoxygen>`_.

Anschließend wird der Code mit Kommentaren von einem Converter (Doxygen) in eine Dokumenatition verwandelt. Dabei gilt: Je umfangreicher und informativer die Kommentare, desto besser die Dokumentation!

Der Prozess des Konvertierens funktioniert so, dass man sich doxygen runterlädt (gibt es unter diesem Name in den 14.04 repos).

Anschließend generiert man sich vor dem Konvertieren mit `doxygen -g [NAME DER KONFIGURATIONSDATEI]` eine Beispielkonfigurationsdatei. Diese sollte noch per Hand an die Anforderungen angepasst werden,
wobei vor allem wichtig ist welche Dateien überhaupt konvertiert werden. Die Konfigurationsdatei ist ziemlich gut kommentiert, sollte also alles selbsterklärend sein.

Ist man damit fertig, kann das konvertieren beginnen mit `dogygen [NAME DER KONFIGURATIONSDATEI]`. Anschließend ist die Konfiguration einsehbar in Unterordnern die nach den verwendeten Formaten benannt sind.
