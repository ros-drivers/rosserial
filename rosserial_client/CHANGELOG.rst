^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2020-08-25)
------------------
* Method to compute duration between time stamps (`#498 <https://github.com/ros-drivers/rosserial/issues/498>`_)
* Python 3 and GCC7+ fixes (`#508 <https://github.com/ros-drivers/rosserial/issues/508>`_)
  * Fixes for new warnings/behaviours in GCC 7+.
  * Resolve uninitMemberVar warnings in NodeHandle.
  * Use nullptr consistently.
* Correctly handle seconds overflow in client time (`#497 <https://github.com/ros-drivers/rosserial/issues/497>`_)
* Use os.path.join for path concatenation (`#495 <https://github.com/ros-drivers/rosserial/issues/495>`_)
* Do not fail when trying to update ros_lib which already exists (`#494 <https://github.com/ros-drivers/rosserial/issues/494>`_)
* ros_lib: Removed code duplication in node_handle.h (`#483 <https://github.com/ros-drivers/rosserial/issues/483>`_)
* AVR Float64: fix cases like denormal numbers & inf (`#442 <https://github.com/ros-drivers/rosserial/issues/442>`_)
* Bump minimum CMake version to 3.7.2 (Melodic).
* Change the access modifiers in NodeHandle\_ from "private" to "protected" to make it easier to derive. (`#426 <https://github.com/ros-drivers/rosserial/issues/426>`_)
* Small Python 3 fixes for rosserial scripts. (`#420 <https://github.com/ros-drivers/rosserial/issues/420>`_)
* Contributors: Asuki Kono, Hermann von Kleist, Mike Purvis, Petteri Aimonen, Rik Baehnemann, 趙　漠居(Zhao, Moju)

0.8.0 (2018-10-11)
------------------
* Add an empty newline to the generated headers. (`#389 <https://github.com/ros-drivers/rosserial/issues/389>`_)
* Add support for boolean parameters (`#355 <https://github.com/ros-drivers/rosserial/issues/355>`_)
* Contributors: Miklós Márton, Pikrass

0.7.7 (2017-11-29)
------------------
* Add overall spin timeout to rosserial read. (`#334 <https://github.com/ros-drivers/rosserial/issues/334>`_)
* Fixing formatting on files. (`#333 <https://github.com/ros-drivers/rosserial/issues/333>`_)
* Fix catkin lint errors (`#296 <https://github.com/ros-drivers/rosserial/issues/296>`_)
* fix spinOnce timeout : 5ms -> 5s (`#326 <https://github.com/ros-drivers/rosserial/issues/326>`_)
* [Client] Fix a warning in comparison. (`#323 <https://github.com/ros-drivers/rosserial/issues/323>`_)
* Use const in ros namespace instead of #define for constants. Fix `#283 <https://github.com/ros-drivers/rosserial/issues/283>`_ (`#318 <https://github.com/ros-drivers/rosserial/issues/318>`_)
* Fix CMP0046 warnings in catkin-built firmwares (`#320 <https://github.com/ros-drivers/rosserial/issues/320>`_)
* Prevent time variable overflow leading to parameter timeout error (`#293 <https://github.com/ros-drivers/rosserial/issues/293>`_)
* Add class member method callback support for Service Server. (`#282 <https://github.com/ros-drivers/rosserial/issues/282>`_)
* Added capability to specify timeout in getParam methods (`#278 <https://github.com/ros-drivers/rosserial/issues/278>`_)
* Contributors: 1r0b1n0, Alessandro Francescon, Bei Chen Liu, David Portugal, Dmitry Kargin, Mike O'Driscoll, Mike Purvis, Romain Reignier

0.7.6 (2017-03-01)
------------------
* Fixing message has no attribute _md5sum (`#257 <https://github.com/ros-drivers/rosserial/issues/257>`_)
* Contributors: Mike O'Driscoll

0.7.5 (2016-11-22)
------------------
* rosserial client variable typedefs (`#254 <https://github.com/ros-drivers/rosserial/issues/254>`_)
  * Add typedefs to generated messages
  This brings rosserial message headers in line with
  roscpp headers that provide a typedef for the message
  variable member.
  * Removing unused imports and variables.
* Added functions for endian-agnostic memory copying (`#240 <https://github.com/ros-drivers/rosserial/issues/240>`_)
* Contributors: Mike O'Driscoll, ivan

0.7.4 (2016-09-21)
------------------
* Integration tests for rosserial (`#243 <https://github.com/ros-drivers/rosserial/issues/243>`_)
* Support member functions as subscriber callbacks.
* Contributors: Mike O'Driscoll, Mike Purvis

0.7.3 (2016-08-05)
------------------
* Order packages by alpha rather than topologically. (`#234 <https://github.com/ros-drivers/rosserial/issues/234>`_)
* Contributors: Mike Purvis

0.7.2 (2016-07-15)
------------------
* Add ros.h include to transform_broadcaster
* Add environment variable for arduino location
* Supported 32bit array lengths in python make_libraries script
* Contributors: Alan Meekins, David Lavoie-Boutin

0.7.1 (2015-07-06)
------------------
* Provide option to pass through CMake arguments in the CMAKE_COMMAND
  invocation. The use-case is primarily specifying additional paths to
  modules, for separately-packaged libraries.
* Contributors: Mike Purvis

0.7.0 (2015-04-23)
------------------
* Initial release for Jade.
* Make message generating error message more verbose.
* Fix initializer for fixed-length arrays.
* Generate constructors for messages.
* Switch to stdint integers. This allows the client to run on 64-bit systems.
* Contributors: Mickaël, Mike Purvis, Mitchell Wills, chuck-h

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
