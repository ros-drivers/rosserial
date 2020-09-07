^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2020-08-25)
------------------
* Only initialize embedded python interpreter once. (`#491 <https://github.com/ros-drivers/rosserial/issues/491>`_)
* Port 482 and 483 forward from Melodic branch (`#492 <https://github.com/ros-drivers/rosserial/issues/492>`_)
* Fix warning when using std_msgs/Empty (`#482 <https://github.com/ros-drivers/rosserial/issues/482>`_)
* Bump minimum CMake version to 3.7.2 (Melodic).
* Removed unused service client for message info service (`#481 <https://github.com/ros-drivers/rosserial/issues/481>`_)
* Call io_service.stop() when ros::ok() returns false (`#477 <https://github.com/ros-drivers/rosserial/issues/477>`_)
* Call Py_Finalize before throwing exception (`#476 <https://github.com/ros-drivers/rosserial/issues/476>`_)
* [Windows] use c++ signed trait to replace ssize_t for better portability. (`#463 <https://github.com/ros-drivers/rosserial/issues/463>`_)
* Port rosserial_server to Boost 1.71. (`#468 <https://github.com/ros-drivers/rosserial/issues/468>`_)
* rosserial_server: update install rules for binary targets (`#457 <https://github.com/ros-drivers/rosserial/issues/457>`_)
* Fix bug: assign the md5 for service (`#449 <https://github.com/ros-drivers/rosserial/issues/449>`_)
* Contributors: Hermann von Kleist, Johannes Meyer, Mike Purvis, Sean Yen, 趙　漠居(Zhao, Moju)

0.8.0 (2018-10-11)
------------------
* Fix compiling on boost > 1.66 (`#362 <https://github.com/ros-drivers/rosserial/issues/362>`_)
  Reflective of changes made to boost::asio noted here:
  http://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/net_ts.html
* Contributors: Fan Jiang

0.7.7 (2017-11-29)
------------------
* Fix catkin lint errors (`#296 <https://github.com/ros-drivers/rosserial/issues/296>`_)
* Contributors: Bei Chen Liu

0.7.6 (2017-03-01)
------------------

0.7.5 (2016-11-22)
------------------
* Fixing build errors for boost >=1.60 (`#226 <https://github.com/ros-drivers/rosserial/issues/226>`_) (`#250 <https://github.com/ros-drivers/rosserial/issues/250>`_)
* Contributors: Malte Splietker

0.7.4 (2016-09-21)
------------------
* Use catkin_EXPORTED_TARGETS to avoid CMake warning (`#246 <https://github.com/ros-drivers/rosserial/issues/246>`_)
* Fix AsyncReadBuffer for UDP socket case. (`#245 <https://github.com/ros-drivers/rosserial/issues/245>`_)
* Contributors: Mike Purvis

0.7.3 (2016-08-05)
------------------
* Avoid runaway async condition when port is bad. (`#236 <https://github.com/ros-drivers/rosserial/issues/236>`_)
* Add missing install rule for udp_socket_node
* Make the ~require param configurable from Session. (`#233 <https://github.com/ros-drivers/rosserial/issues/233>`_)
* Contributors: Mike Purvis

0.7.2 (2016-07-15)
------------------
* Implementation of native UDP rosserial server. (`#231 <https://github.com/ros-drivers/rosserial/issues/231>`_)
* Explicit session lifecycle for the serial server. (`#228 <https://github.com/ros-drivers/rosserial/issues/228>`_)
  This is a long overdue change which will resolve some crashes when
  USB serial devices return error states in the face of noise or other
  interruptions.
* Support for VER1 protocol has been dropped.
* Handle log messages in rosserial_server
* Contributors: Mike Purvis, mkrauter

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
