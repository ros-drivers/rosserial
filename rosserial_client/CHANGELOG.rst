^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
