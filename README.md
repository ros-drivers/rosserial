## ROSserial for Groovy+

This repo is ported from https://kforge.ros.org/rosserial/hg. It is in the process of being catkinized and updated for ROS Groovy and newer.

## Changes for Groovy/Catkin

 * Moved to catkin build system 
 * Moved rosserial_xbee to experimential stack
 * Created new rosserial metapackage, with depends only on python, client and msgs.
 * rosserial_arduino and rosserial_embeddedlinux are not generating messages right now, will be fixed shortly.
 * Closing tickets:
   * kforge #77: missing newline
   * kforge #78: WProgram.h name
