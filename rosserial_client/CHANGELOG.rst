^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.4 (2014-11-05)
------------------
* Updated header size to 16 bits to allow larger data files for Jade
* Contributors: Ilia Baranov

0.6.3 (2014-11-05)
------------------
* Move avr serialization logic to Msg class, add gtest to exercise it.
* Fixed the deserialization of avr64bit in order to support negative numbers.
* Contributors: Martin Gerdzhev, Mike Purvis

0.6.2 (2014-09-10)
------------------
* Generic CMake helpers for ros_lib generation and in-package firmwares.
* Fix output of make_library when package has only messages
* Added time out to the state machine
* Contributors: Jason Scatena, Michael Ferguson, Mike Purvis

0.6.1 (2014-06-30)
------------------
* Remove ID_TX_STOP define
* Fix ID_TX_STOP in the client lib.
* Contributors: Mike Purvis

0.6.0 (2014-06-11)
------------------
* Remove include of ros.h from time.cpp
* No xx_val pointers for fixed-length arrays of messages.
* Use const char* instead of char* for strings in messages.
* Contributors: Mike Purvis

0.5.6 (2014-06-11)
------------------
* Add Mike Purvis as maintainer
* make tf topic absolute instead of relative to prevent remapping with <group> tag
* fix: msg id serialization
* fix: wrong message lenght, if message size more than 255
* fix odometry deserialization error http://answers.ros.org/question/73807/rosserial-deserialization-error/
* add better debugging information when packages are missing dependencies
* remove ID_TX_STOP from rosserial_msgs/msg/TopicInfo.msg, using hardcode modification.
* fix the dupilcated registration problem of subscriber
* Contributors: Michael Ferguson, Mike Purvis, Moju Zhao, agentx3r, denis

0.5.5 (2014-01-14)
------------------

0.5.4 (2013-10-17)
------------------
* fix an uninitialized data bug on arduino

0.5.3 (2013-09-21)
------------------
* Added some missing return values
* Fixed uninitialized arrays that would cause random segfaults on spinOnce 
and advertise. Fixed other ininitialized variables.
* fixed misalignment for 32 bit architectures

0.5.2 (2013-07-17)
------------------

* Fix release version

0.5.1 (2013-07-15)
------------------
* Modified the return value of publish()
* Modified the frame structure for serial communication, particularly add the checksum for msg_len
  * Associated protocol version ID in message and version mismatch handling

0.4.5 (2013-07-02)
------------------
* fail gently when messages/packages are corrupt. update print statements while at it
* Fixed a bug in ros_lib install logic which took an exception because it copied files to themselves
  Added execute permission to make_libraries.py in rosserial_embeddedlinux
  Moved examples under src in rosserial_embeddedlinux

0.4.4 (2013-03-20)
------------------

0.4.3 (2013-03-13 14:08)
------------------------

0.4.2 (2013-03-13 01:15)
------------------------
* fix build issues when in isolation by moving more stuff into make_library

0.4.1 (2013-03-09)
------------------

0.4.0 (2013-03-08)
------------------
* initial catkin version on github
* Temporary patch for `#30 <https://github.com/ros-drivers/rosserial/issues/30>`_
* Added missing math.h include.
* Changed DEBUG log level to ROSDEBUG.
