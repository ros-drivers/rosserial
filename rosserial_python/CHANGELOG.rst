^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
