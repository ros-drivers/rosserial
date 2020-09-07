^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_arduino
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2020-08-25)
------------------
* Install arduino_python scripts to bin destination (`#506 <https://github.com/ros-drivers/rosserial/issues/506>`_)
* Use os.path.join for path concatenation (`#495 <https://github.com/ros-drivers/rosserial/issues/495>`_)
* Added target ID to support Teensy 4.0 (`#460 <https://github.com/ros-drivers/rosserial/issues/460>`_)
* Bump minimum CMake version to 3.7.2 (Melodic).
* Allow user code to change Arduino port used by client (`#472 <https://github.com/ros-drivers/rosserial/issues/472>`_)
* Avoid looping over write buffer in Arduino client. (`#475 <https://github.com/ros-drivers/rosserial/issues/475>`_)
* Fix py3 print usages and trailing whitespaces (`#469 <https://github.com/ros-drivers/rosserial/issues/469>`_)
* Contributors: André Araújo, D. Petrini, Hermann von Kleist, Mike Purvis, Rik Baehnemann, acxz, schnaubelt

0.8.0 (2018-10-11)
------------------
* Enhance ArduinoTcpHardware (`#379 <https://github.com/ros-drivers/rosserial/issues/379>`_)
  - Add support to check for current TCP connection status to reconnect
  - stop tcp before re-connecting
* Changed hardcoded pin 13 to LED_BUILTIN (`#328 <https://github.com/ros-drivers/rosserial/issues/328>`_)
  To make the code work on an Arduino Nano V3 clone board.
  (might need to be updated in BlinkM as well, but I didnt try that example)
* Added service to force an Arduino hard reset in serial_node.py (`#349 <https://github.com/ros-drivers/rosserial/issues/349>`_)
  * Added hard_reset service call to serial_node
  * Refactored SerialClient to use a write thread, working around deadlock when both Arduino and serial_node.py get stuck writing to each other.
  * Updated cmakelists and package.xml to include dependencies. Removed unnecessary tcp functionality from arduino-specific serial_node.py
* Added ESP32 support (`#345 <https://github.com/ros-drivers/rosserial/issues/345>`_)
* In rosserial_arduino, changed embedded type size for ROS uint64 to 8 bytes from 4. (`#312 <https://github.com/ros-drivers/rosserial/issues/312>`_)
* Contributors: Bo Chen, Chris Spencer, Pornthep Preechayasomboon, Tom O'Connell, blubbi321

0.7.7 (2017-11-29)
------------------
* Fix catkin lint errors (`#296 <https://github.com/ros-drivers/rosserial/issues/296>`_)
* Add ArduinoTcpHardware to use Arduino Ethernet shield. (`#324 <https://github.com/ros-drivers/rosserial/issues/324>`_)
* Add an example to use Subscriber and ServiceServer as class members. (`#321 <https://github.com/ros-drivers/rosserial/issues/321>`_)
* Added support for the Particle Photon (`#292 <https://github.com/ros-drivers/rosserial/issues/292>`_)
* Fix POLICY CMP0054 unknown (`#291 <https://github.com/ros-drivers/rosserial/issues/291>`_)
* Use ESP8266 header only if defined. (`#288 <https://github.com/ros-drivers/rosserial/issues/288>`_)
* Add Esp8266 support and an example (`#279 <https://github.com/ros-drivers/rosserial/issues/279>`_)
* Add support for STM32F1 with stm32duino. (`#281 <https://github.com/ros-drivers/rosserial/issues/281>`_)
* Contributors: Bei Chen Liu, David Portugal, Dmitry Kargin, Mike Purvis, Romain Reignier, Valentin VERGEZ, khancyr

0.7.6 (2017-03-01)
------------------
* Fixed issue with CMake CMP0054 (`#273 <https://github.com/ros-drivers/rosserial/issues/273>`_)
* Add Teensy LC support (`#270 <https://github.com/ros-drivers/rosserial/issues/270>`_)
* Support Teensy 3.5, 3.6. (`#259 <https://github.com/ros-drivers/rosserial/issues/259>`_)
* Contributors: Brent Yi, FirefoxMetzger, Mike Purvis

0.7.5 (2016-11-22)
------------------
* Missing 'h' inside constructor ArduinoHardware(ArduinoHardware& h) (`#251 <https://github.com/ros-drivers/rosserial/issues/251>`_)
* Contributors: MalcolmReynlods

0.7.4 (2016-09-21)
------------------

0.7.3 (2016-08-05)
------------------

0.7.2 (2016-07-15)
------------------
* Add ros.h include to transform_broadcaster
* Add environment variable for arduino location
* Add support for HW Serial ports on the Teensy
* Contributors: David Lavoie-Boutin, Gary Servin

0.7.1 (2015-07-06)
------------------

0.7.0 (2015-04-23)
------------------

0.6.3 (2014-11-05)
------------------
* Fix for Arduino upload path issue.
* Contributors: Mike Purvis

0.6.2 (2014-09-10)
------------------
* Clean up rosserial_arduino/package.xml
* Generic CMake helpers.
* Contributors: Mike Purvis

0.6.1 (2014-06-30)
------------------

0.6.0 (2014-06-11)
------------------

0.5.6 (2014-06-11)
------------------
* Add Mike Purvis as maintainer
* Updated examples for Arduino 1.+
* Added Teensy 3.1 support (MK20DX256)
* Updated ArduinoHardware.h to add Teensy 3.0 support
* Contributors: Michael Ferguson, Mike Purvis, Moju Zhao, Tony Baltovski, kjanesch

0.5.5 (2014-01-14)
------------------
* Leonardo: Use the USB serial port for ROS messages option


0.5.3 (2013-09-21)
------------------
* add support for leonardo and due

0.5.2 (2013-07-17)
------------------

* Fix release version

0.5.1 (2013-07-15)
------------------

0.4.5 (2013-07-02)
------------------
* Fixed a bug in ros_lib install logic which took an exception because it copied files to themselves
  Added execute permission to make_libraries.py in rosserial_embeddedlinux
  Moved examples under src in rosserial_embeddedlinux
* fix package name

0.4.4 (2013-03-20)
------------------

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
