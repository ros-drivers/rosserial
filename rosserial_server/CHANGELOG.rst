^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2014-06-11)
------------------

0.5.6 (2014-06-11)
------------------
* Fixed build error due to variable being read as a function due to missing parenthesis
* Add rosserial_python as dependency of rosserial_server
* Contributors: Mike Purvis, spaghetti-

0.5.5 (2014-01-14)
------------------
* Add support for require/publishers and require/subscribers parameters.
* Use stream logging in rosserial_server

0.5.4 (2013-10-17)
------------------

0.5.3 (2013-09-21)
------------------
* New package: rosserial_server
* Contains example launch file for serial configuration of server
* Working now with both Groovy and Hydro clients.
* Subscriber to correctly declare known md5 and topic type from client.
