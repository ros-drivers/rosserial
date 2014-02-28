## ROSserial for Hydro

This repo is ported from https://kforge.ros.org/rosserial/hg. It has been catkinized and updated for ROS Groovy and newer.

## Changes for Hydro

0.5.5
 * Add support for Arduino Yum
 * Release rosserial_server

0.5.4
 * Fix a bug in string deserialization where part of length field could be uninitialized
(issues 74 & 76)

0.5.3
 * Add rosserial_server - a C++ server implementation
 * Fix misc bugs including adding fixes for alignment issues on ARM processors
0.5.2
 * Wire protocol change to add checksum to message length
 * Support empty requests
 * Re-integrate rosserial_xbee

## Changes for Groovy/Catkin

 * Moved to catkin build system 
 * Moved rosserial_xbee to experimential stack
 * Created new rosserial metapackage, with depends only on python, client and msgs.
 * Closed, integrated and/or moved all tickets to github.
 * New message generation and workflow:
   * no longer uses roslib.rospack, or roslib.gentools (and is way, WAY faster)
   * messages are no longer built at built-time (yeah, sounds ridiculous, but is true, see below for new workflow)
   * beginnings of easier porting to new platforms (the lookup table is now found in architecture-dependent packages, etc)

## Usage/Workflow
Workflow is a bit different from in the past. Rather than running the library generator over each package you want to use, you run it once and generate libraries for all installed messages:

      cd <ws>/src
      git clone https://github.com/ros-drivers/rosserial
      cd <ws>
      catkin_make
      catkin_make install
      source <ws>/install/setup.bash
      cd <sketchbook>/libraries
      rosrun rosserial_arduino make_libraries.py .

Note: currently you will HAVE to run catkin_make install, otherwise portions of the ros_lib directory will be missing. This will hopefully be fixed soon. Also note: you may have to delete <sketchbook>/libraries/ros_lib in order to regenerate as its existence may cause an error.
