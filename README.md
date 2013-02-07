## ROSserial for Groovy+

This repo is ported from https://kforge.ros.org/rosserial/hg. It is in the process of being catkinized and updated for ROS Groovy and newer.

## Changes for Groovy/Catkin

 * Moved to catkin build system (still to do: rosserial_arduino, rosserial_embeddedlinux, rosserial_xbee)
 * Created new rosserial metapackage, with depends only on python, client and msgs.
