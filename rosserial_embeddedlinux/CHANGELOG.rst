^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_embeddedlinux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.5 (2014-01-14)
------------------

0.5.4 (2013-10-17)
------------------

0.5.3 (2013-09-21)
------------------

0.5.2 (2013-07-17)
------------------

* Fix release version

0.5.1 (2013-07-15)
------------------

0.4.5 (2013-07-02)
------------------
* rosserial_embeddedlinux: Fix CMakeLists.txt ...
  so 'catkin_make install' can find 'examples' directory under 'src' directory in rosserial_embeddedlinux
* fix install rule for examples dir
* Fixed a bug in ros_lib install logic which took an exception because it copied files to themselves
  Added execute permission to make_libraries.py in rosserial_embeddedlinux
  Moved examples under src in rosserial_embeddedlinux
* fix package name

0.4.4 (2013-03-20)
------------------
* merge
* Merge remote-tracking branch 'origin/groovy-devel' into groovy-devel
* Install examples in install dir & next to ros_lib. Change maintainer email.

0.4.3 (2013-03-13 14:08)
------------------------
* forgot to remove install directives

0.4.2 (2013-03-13 01:15)
------------------------
* fix build issues when in isolation by moving more stuff into make_library

0.4.1 (2013-03-09)
------------------

0.4.0 (2013-03-08)
------------------
* initial catkin version on github
