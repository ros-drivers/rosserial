#!/usr/bin/env python

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

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import roslib; roslib.load_manifest("rosserial_python")
import rospy

import thread
from serial import *
import StringIO

from std_msgs.msg import Time
from rosserial_msgs.msg import *

MODE_FIRST_FF = 0
MODE_SECOND_FF = 1

TOPIC_PUBLISHERS = 0
TOPIC_SUBSCRIBERS = 1
TOPIC_TIME = 10

class Publisher:
    """ 
        Prototype of a forwarding publisher.
    """

    def __init__(self, topic, message_type):
        """ Create a new publisher. """ 
        self.topic = topic
        
        # find message type
        package, message = message_type.split('/')
        try:
            m = __import__( package +'.msg')
        except:
            roslib.load_manifest(package)
            m = __import__( package +'.msg')
        m2 = getattr(m, 'msg')
        self.message = getattr(m2, message)
        self.publisher = rospy.Publisher(topic, self.message)
    
    def publish(self, msg):
        """ Publish a message """ 
        m = self.message()
        m.deserialize(msg)
        self.publisher.publish(m)
                         

class Subscriber:
    """ 
        Prototype of a forwarding subscriber.
    """

    def __init__(self, topic, message_type, parent):
        self.topic = topic
        self.parent = parent
        
        # find message type
        package, message = message_type.split('/')
        try:
            m = __import__( package +'.msg')
        except:
            roslib.load_manifest(package)
            m = __import__( package +'.msg')
        m2 = getattr(m, 'msg')
        self.message = getattr(m2, message)
        rospy.Subscriber(topic, self.message, self.callback)

    def callback(self, msg):
        """ Forward a message """
        data_buffer = StringIO.StringIO()
        msg.serialize(data_buffer)
        self.parent.send(self.parent.subscribers[self.topic][0], data_buffer.getvalue())


class SerialClient:
    """
        Prototype of rosserial python client to connect to serial bus.
    """
    
    def __init__(self, port=None, baud=57600):
        """ Initialize node, connect to bus, attempt to negotiate topics. """
        self.mutex = thread.allocate_lock()

        if port== None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
			#assume its a filelike object
			self.port=port
        else:
            # open a specific port
            self.port = Serial(port, baud)
            
        self.publishers = dict()
        self.subscribers = dict()
        rospy.sleep(1.0) # TODO
        self.requestTopics()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        self.port.flushInput()
        # request topic sync
        self.port.write("\xff\xff\x00\x00\x00\x00\xff")
	
    def run(self):
        """ Forward recieved messages to appropriate publisher. """
        mode = MODE_FIRST_FF
        while not rospy.is_shutdown():
            if mode == MODE_FIRST_FF:
                if self.port.read() == '\xff':
                    mode += 1
            elif mode == MODE_SECOND_FF:
                x = self.port.read()
                if x == '\xff':
                    # topic id (2 bytes)
                    topic_id = ord(self.port.read())
                    checksum = topic_id
                    temp = ord(self.port.read())
                    checksum += temp
                    topic_id += temp<<8
                    # message length (2 bytes)
                    bytes = ord(self.port.read())       
                    checksum += bytes
                    temp = ord(self.port.read())
                    bytes += temp<<8
                    checksum += temp    
                    # message
                    msg = self.port.read(bytes)
                    checksum += sum( [ord(x) for x in msg] )
                    checksum += ord( self.port.read() )
                    # checksum                    
                    if checksum%256 == 255:
                        if topic_id == TOPIC_PUBLISHERS:
                            try:
                                m = TopicInfo()
                                m.deserialize(msg)
                                self.publishers[m.topic_id] = Publisher(m.topic_name, m.message_type)
                                rospy.loginfo("Setup Publisher on %s [%s]" % (m.topic_name, m.message_type) )

                            except:
                                rospy.logerr("Failed to parse publisher.")

                        elif topic_id == TOPIC_SUBSCRIBERS:
                            try:
                                m = TopicInfo()
                                m.deserialize(msg)
                                self.subscribers[m.topic_name] = [m.topic_id, Subscriber(m.topic_name, m.message_type, self)]
                                rospy.loginfo("Setup Subscriber on %s [%s]" % (m.topic_name, m.message_type))
                            except:
                                rospy.logerr("Failed to parse subscriber.")

                        elif topic_id == TOPIC_TIME:
                            t = Time()
                            t.data = rospy.Time.now()
                            data_buffer = StringIO.StringIO()
                            t.serialize(data_buffer)
                            self.send( TOPIC_TIME, data_buffer.getvalue() )

                        elif topic_id >= 100: # TOPIC
                            try:
                                self.publishers[topic_id].publish(msg)
                            except KeyError:
                                rospy.logerr("Tried to publish before configured, topic id %d" % topic_id)

                        else:
                            rospy.logerr("Unrecognized command topic!")
                        
                    else: # end if checksum == 255
                        rospy.logerr("Failed checksum")

                else: # end if 0xff
                    rospy.loginfo("Failed packet")

                mode = MODE_FIRST_FF
            
    def send(self, topic, msg):
        """ Send a message on a particular topic to the device. """
        with self.mutex:
            length = len(msg)
            checksum = 255 - ( ((topic&255) + (topic>>8) + (length&255) + (length>>8) + sum([ord(x) for x in msg]))%256 )
            self.port.write('\xff\xff')
            self.port.write(chr(topic&255))
            self.port.write(chr(topic>>8))
            self.port.write(chr(length&255))
            self.port.write(chr(length>>8))
            self.port.write(msg)
            self.port.write(chr(checksum))

