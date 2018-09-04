#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import time

import rospy
from std_srvs.srv import Empty, EmptyResponse

from serial import Serial

from rosserial_python import SerialClient as _SerialClient

MINIMUM_RESET_TIME = 30

class SerialClient(_SerialClient):

    def __init__(self, *args, **kwargs):
        # The number of seconds to wait after a sync failure for a sync success before automatically performing a reset.
        # If 0, no reset is performed.
        self.auto_reset_timeout = kwargs.pop('auto_reset_timeout', 0)
        self.lastsync_reset = rospy.Time.now()
        rospy.Service('~reset_arduino', Empty, self.resetArduino)
        super(SerialClient, self).__init__(*args, **kwargs)

    def resetArduino(self, *args, **kwargs):
        """
        Forces the Arduino to perform a reset, as though its reset button was pressed.
        """
        with self.read_lock, self.write_lock:
            rospy.loginfo('Beginning Arduino reset on port %s. Closing serial port...' % self.port.portstr)
            self.port.close()
            with Serial(self.port.portstr) as arduino:
                arduino.setDTR(False)
                time.sleep(3)
                arduino.flushInput()
                arduino.setDTR(True)
                time.sleep(5)
            rospy.loginfo('Reopening serial port...')
            self.port.open()
            rospy.loginfo('Arduino reset complete.')
            self.lastsync_reset = rospy.Time.now()
        self.requestTopics()
        return EmptyResponse()

    def sendDiagnostics(self, level, msg_text):
        super(SerialClient, self).sendDiagnostics(level, msg_text)
        # Reset when we haven't received any data from the Arduino in over N seconds.
        if self.auto_reset_timeout and (rospy.Time.now() - self.last_read).secs >= self.auto_reset_timeout:
            if (rospy.Time.now() - self.lastsync_reset).secs < MINIMUM_RESET_TIME:
                rospy.loginfo('Sync has failed, but waiting for last reset to complete.')
            else:
                rospy.loginfo('Sync has failed for over %s seconds. Initiating automatic reset.' % self.auto_reset_timeout)
                self.resetArduino()
