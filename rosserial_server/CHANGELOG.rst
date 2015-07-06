^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2015-07-06)
------------------

0.7.0 (2015-04-23)
------------------
* Fill out description field in package.xml.
* Bugfix for checksum.
  Publishing topics fails when messages are over 256 bytes in length due to checksum() function or'ing high and low byte instead of adding them.
* rosserial_server: Properly receive messages > 255 bytes.
* Contributors: Chad Attermann, Mike Purvis

0.6.3 (2014-11-05)
------------------
* Add more log output, don't end the session for certain write errors.
* Contributors: Mike Purvis

0.6.2 (2014-09-10)
------------------
* Bugfix for interrupted sessions.
  This is a two-part fix for an issue causes a segfault when the device
  disappears during operation, for example a ttyACM device which is unplugged.
  The AsyncReadBuffer part avoids calling a callback after the object
  owning it has destructed, and the SerialSession part avoids recreating
  itself until the previous instance has finished the destructor and been
  full destroyed.
* Add dependency on rosserial_msgs_gencpp, fixes `#133 <https://github.com/ros-drivers/rosserial/issues/133>`_
* Make ServiceClient::handle public, to fix compilation bug on some platforms.
* Enabled registration of service clients
* Add namespaces to headers, swap ROS thread to foreground.
* Move headers to include path, rename to follow ROS style.

0.6.1 (2014-06-30)
------------------

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
