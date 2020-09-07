^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2020-08-25)
------------------
* Python 3 and GCC7+ fixes (`#508 <https://github.com/ros-drivers/rosserial/issues/508>`_)
  * Port of rosserial_python to py3.
  * Throw from inside the BrokenPipeError.
* Fix Travis for Noetic + Python 3
* Bump minimum CMake version to 3.7.2 (Melodic).
* Update pyserial rosdep.
* Use time.sleep instead of rospy.sleep. (`#489 <https://github.com/ros-drivers/rosserial/issues/489>`_)
* Make deprecation message a warning and more specific (`#479 <https://github.com/ros-drivers/rosserial/issues/479>`_)
* Properly initialize message_info stub, drop from test.
* Fix py3 print usages and trailing whitespaces (`#469 <https://github.com/ros-drivers/rosserial/issues/469>`_)
* Drop separate node for message service (`#446 <https://github.com/ros-drivers/rosserial/issues/446>`_)
* Fix reconnection of rosserial-python (`#445 <https://github.com/ros-drivers/rosserial/issues/445>`_)
* Contributors: Asuki Kono, Daisuke Sato, Hermann von Kleist, Hikaru Sugiura, Mike Purvis, acxz

0.8.0 (2018-10-11)
------------------
* fix no attribute message_cache issue in message_info_service (`#393 <https://github.com/ros-drivers/rosserial/issues/393>`_)
* Added service to force an Arduino hard reset in serial_node.py (`#349 <https://github.com/ros-drivers/rosserial/issues/349>`_)
  * Added hard_reset service call to serial_node
  * Refactored SerialClient to use a write thread, working around deadlock when both Arduino and serial_node.py get stuck writing to each other.
  * Updated cmakelists and package.xml to include dependencies. Removed unnecessary tcp functionality from arduino-specific serial_node.py
* Add support for boolean parameters (`#355 <https://github.com/ros-drivers/rosserial/issues/355>`_)
* [python] fix an unboundlocalerror (`#346 <https://github.com/ros-drivers/rosserial/issues/346>`_)
* Retry opening the serial port every 3 seconds (`#342 <https://github.com/ros-drivers/rosserial/issues/342>`_)
  * Retry opening the serial port every 3 seconds
  * Break out of the retry loop if we've been shut down
* Contributors: Chris Spencer, Kenta Yonekura, Pikrass, dlguo-cpr

0.7.7 (2017-11-29)
------------------
* Fix catkin lint errors (`#296 <https://github.com/ros-drivers/rosserial/issues/296>`_)
* pyserial bug workaround to fix rosserial_test test against SerialClient.py (`#313 <https://github.com/ros-drivers/rosserial/issues/313>`_)
* Add requestTopics to correct publish before setup (`#308 <https://github.com/ros-drivers/rosserial/issues/308>`_)
* Contributors: Bei Chen Liu, Tom O'Connell

0.7.6 (2017-03-01)
------------------
* Fix typo in serial error message (`#253 <https://github.com/ros-drivers/rosserial/issues/253>`_)
* Contributors: Jonathan Binney

0.7.5 (2016-11-22)
------------------

0.7.4 (2016-09-21)
------------------
* Try to read more serial bytes in a loop (`#248 <https://github.com/ros-drivers/rosserial/issues/248>`_)
* Add missing "import errno" to rosserial_python
* Integration tests for rosserial (`#243 <https://github.com/ros-drivers/rosserial/issues/243>`_)
* rosserial_python tcp server allowing socket address reuse (`#242 <https://github.com/ros-drivers/rosserial/issues/242>`_)
* Contributors: Mike Purvis, Vitor Matos, davidshin172

0.7.3 (2016-08-05)
------------------

0.7.2 (2016-07-15)
------------------

0.7.1 (2015-07-06)
------------------

0.7.0 (2015-04-23)
------------------
* Adds default queue_size of 10 for rosserial_python publisher.
* Fixed queue size warning with diagnostics publisher.
* We don't need roslib.load_manifest any more under catkin.
* Contributors: Basheer Subei, David Lavoie-Boutin, Mike Purvis, eisoku9618

0.6.3 (2014-11-05)
------------------

0.6.2 (2014-09-10)
------------------
* Added MD5 verification for request and response messags upon ServiceClient registration.
* Enabled registration of service clients
* Contributors: Jonathan Jekir

0.6.1 (2014-06-30)
------------------

0.6.0 (2014-06-11)
------------------

0.5.6 (2014-06-11)
------------------
* Add Mike Purvis as maintainer to all but xbee.
* Added the missing inWaiting() to RosSerialServer
* improvement: inform user of mismatched checksum for topic_id and msg
* Fix indent on if-block.
* Check for data in buffer before attempting to tryRead. Insert a 1ms sleep to avoid pegging the processor.
* Better warning message for tryRead.
* fix two points: 1. the number of bytes to read for chk_byte, 2. the wrong indentation about the defination of sendDiagnostics()
* Try-block to handle IOErrors thrown from tryRead
* Merge from hydro-devel.
* fix the dupilcated registration problem of subscriber
* remove ID_TX_STOP from rosserial_msgs/msg/TopicInfo.msg, using hardcode modification. fix the dupilcated registration problem of subscriber
* modified rosserial
* modified rosserial
* Contributors: Girts Linde, Mike Purvis, Moju Zhao, bakui, denis

0.5.5 (2014-01-14)
------------------

0.5.4 (2013-10-17)
------------------

0.5.3 (2013-09-21)
------------------
* De-register subscribers and service clients upon disconnect.
  This prevents callbacks being called after a client program
  terminates a connection.
* Fill out package.xml properly, include docstring in helper Python node.
* Add message info helper script that supports rosserial_server

0.5.2 (2013-07-17)
------------------

* Fix release version

0.5.1 (2013-07-15)
------------------
* Merge branch 'rosserial_bakui' of git://github.com/tongtybj/rosserial into tongtybj-rosserial_bakui
  * Modified the frame structure for serial communication, particularly add the checksum for msg_len
* Incorporate protocol version in message. Try to detect protocol version mismatch and give appropriate hints.

0.4.5 (2013-07-02)
------------------
* Fix SeviceServer member names in error message
  'm' prefix was omitted, causing an exception while trying to print
  an error about md5 mismatches. Fix this to allow the error to be
  presented to the user.
* Allow service calls with empty requests
  std_srvs::Empty has a request message of size zero. SerialClient.send
  returns the size of the sent message, which is checked to ensure
  data crossed the serial line. Accommodate services with empty requests
  by modifying the check to acknowledge all transmissions of zero or
  more bytes as valid.
* revert name of node, add a few comments/spacing
* fix private parameters - temporary fix breaks fork_server for tcp
* Fix `#35 <https://github.com/ros-drivers/rosserial/issues/35>`_

0.4.4 (2013-03-20)
------------------
* Fixed "Lost sync" message at initial connection that happens on both Arduino &
  embeddedLinux. Problem was last_sync initialized to epoch and compared against
  Time.now() always times out on first compare.

0.4.3 (2013-03-13 14:08)
------------------------

0.4.2 (2013-03-13 01:15)
------------------------

0.4.1 (2013-03-09)
------------------

0.4.0 (2013-03-08)
------------------
* initial catkin version on github
