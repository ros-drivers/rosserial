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
from rosserial_msgs.srv import *

import time

import struct

MODE_FIRST_FF = 0
MODE_SECOND_FF = 1


def load_pkg_module(package):
    #check if its in the python path
    in_path = False
    for entry in sys.path:
        if package in entry:
            in_path = True
    if not in_path:
        roslib.load_manifest(package)
    try:
        m = __import__( package +'.msg')
    except:
        rospy.logerr( "Cannot import package : %s"% package )
        return None
    return m


class Publisher:
    """ 
        Prototype of a forwarding publisher.
    """
    def __init__(self, topic, message_type):
        """ Create a new publisher. """ 
        self.topic = topic
        
        # find message type
        package, message = message_type.split('/')
        m = load_pkg_module(package)

        m2 = getattr(m, 'msg')
        self.message = getattr(m2, message)
        self.publisher = rospy.Publisher(topic, self.message)
    
    def handlePacket(self, data):
        """ """
        m = self.message()
        m.deserialize(data)
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
        m = load_pkg_module(package)

        m2 = getattr(m, 'msg')
        self.message = getattr(m2, message)
        rospy.Subscriber(topic, self.message, self.callback)

    def callback(self, msg):
        """ Forward a message """
        data_buffer = StringIO.StringIO()
        msg.serialize(data_buffer)
        self.parent.send(self.parent.receivers[self.topic][0], data_buffer.getvalue())

class ServiceServer:
    def __init__(self, name, service_type, parent):
        self.name = service_name
        self.service_type = service_type
        self.parent = parent
        
        # find message type
        package, message = self.service_type.split('/')
        m = load_pkg_module(package)
        
        srvs = getattr(m, 'srv')
        self.srv_req = getattr(srvs, message+"Request")
        self.srv_resp = getattr(srvs, message+"Response")
        self.srv = getattr(srvs, message)
        self.service = rospy.Service(self.service_name, self.srv, self.callback)
        self.response = None
    def handlePacket(self, data):
        resp = self.srv_resp()
        resp.deserialize(data)
        self.response = resp
    def callback(self, msg):
        msg.serialize(data_buffer)
        self.parent.send(self.parent.services[self.name][0], data_buffer.getvalue())
        


class SerialClient:
    """
        Prototype of rosserial python client to connect to serial bus.
    """
    
    def __init__(self, port=None, baud=57600, timeout=5.0):
        """ Initialize node, connect to bus, attempt to negotiate topics. """
        self.mutex = thread.allocate_lock()

        self.lastsync = rospy.Time.now()
        self.timeout = timeout

        if port== None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
            #assume its a filelike object
            self.port=port
        else:
            # open a specific port
            self.port = Serial(port, baud, timeout=self.timeout*0.5)
        
        self.port.timeout = 0.01 #edit the port timeout
        
        time.sleep(0.1) #allow the driver to get ready 
                        #(Important for uno)
        
        self.senders = dict() #Publishers/ServiceServers
        self.receivers = dict() #subscribers/serviceclients
                
        rospy.sleep(2.0) # TODO
        self.requestTopics()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        self.port.flushInput()
        # request topic sync
        self.port.write("\xff\xff\x00\x00\x00\x00\xff")
    

    def run(self):
        """ Forward recieved messages to appropriate publisher. """
        data = ''
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.lastsync).to_sec() > (self.timeout * 3):
                rospy.logerr("Lost sync with device, restarting...")
                self.requestTopics()
                self.lastsync = rospy.Time.now()    
            
            flag = [0,0]
            flag[0]  = self.port.read(1)
            if (flag[0] != '\xff'):
                continue
            flag[1] = self.port.read(1)
            if ( flag[1] != '\xff'):
                rospy.loginfo("Failed Packet Flags ")
                continue
            # topic id (2 bytes)
            header = self.port.read(4)
            if (len(header) != 4):
                #self.port.flushInput()
                continue
            
            topic_id, msg_length = struct.unpack("<hh", header)
            msg = self.port.read(msg_length)
            if (len(msg) != msg_length):
                rospy.loginfo("Packet Failed :  Failed to read msg data")
                #self.port.flushInput()
                continue
            chk = self.port.read(1)
            checksum = sum(map(ord,header) ) + sum(map(ord, msg)) + ord(chk)

            if checksum%256 == 255:
                if topic_id == TopicInfo.ID_PUBLISHER:
                    try:
                        m = TopicInfo()
                        m.deserialize(msg)
                        self.senders[m.topic_id] = Publisher(m.topic_name, m.message_type)
                        rospy.loginfo("Setup Publisher on %s [%s]" % (m.topic_name, m.message_type) )
                    except Exception as e:
                        rospy.logerr("Failed to parse publisher: %s", e)
                elif topic_id == TopicInfo.ID_SUBSCRIBER:
                    try:
                        m = TopicInfo()
                        m.deserialize(msg)
                        self.receivers[m.topic_name] = [m.topic_id, Subscriber(m.topic_name, m.message_type, self)]
                        rospy.loginfo("Setup Subscriber on %s [%s]" % (m.topic_name, m.message_type))
                    except Exception as e:
                        rospy.logerr("Failed to parse subscriber. %s"%e)
                elif topic_id == TopicInfo.ID_SERVICE_SERVER:
                    try:
                        m = TopicInfo()
                        m.deserialize(msg)
                        self.senders[m.topic_id]=ServiceServer(m.topic_name, m.message_type, self) 
                        rospy.loginfo("Setup ServiceServer on %s [%s]"%(m.topic_name, m.message_type) )
                    except:
                        rospy.logerr("Failed to parse service server")
                elif topic_id == TopicInfo.ID_SERVICE_CLIENT:
                    pass
                
                elif topic_id == TopicInfo.ID_PARAMETER_REQUEST:
                    self.handleParameterRequest(msg)
                
                elif topic_id == TopicInfo.ID_LOG:
                    self.handleLogging(msg)
                    
                elif topic_id == TopicInfo.ID_TIME:
                    t = Time()
                    t.data = rospy.Time.now()
                    data_buffer = StringIO.StringIO()
                    t.serialize(data_buffer)
                    self.send( TopicInfo.ID_TIME, data_buffer.getvalue() )
                    self.lastsync = rospy.Time.now()
                elif topic_id >= 100: # TOPIC
                    try:
                        self.senders[topic_id].handlePacket(msg)
                    except KeyError:
                        rospy.logerr("Tried to publish before configured, topic id %d" % topic_id)
                else:
                    rospy.logerr("Unrecognized command topic!")
                rospy.sleep(0.001)
            
    def handleParameterRequest(self,data):
        """Handlers the request for parameters from the rosserial_client
            This is only serves a limmited selection of parameter types.
            It is meant for simple configuration of your hardware. It 
            will not send dictionaries or multitype lists.
        """
        req = RequestParamRequest()
        req.deserialize(data)
        resp = RequestParamResponse()
        param = rospy.get_param(req.name)
        if param == None:
            rospy.logerr("Parameter %s does not exist"%req.name)
            return
        if (type(param) == dict):
            rospy.logerr("Cannot send param %s because it is a dictionary"%req.name)
            return
        if (type(param) != list):
            param = [param]
        #check to make sure that all parameters in list are same type
        t = type(param[0])
        for p in param:
            if t!= type(p):
                rospy.logerr('All Paramers in the list %s must be of the same type'%req.name)
                return      
        if (t == int):
            resp.ints= param
        if (t == float):
            resp.floats=param
        if (t == str):
            resp.strings = param
        print resp
        data_buffer = StringIO.StringIO()
        resp.serialize(data_buffer)
        self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getvalue())
        
        
    def handleLogging(self, data):
        m= Log()
        m.deserialize(data)
        if (m.level == Log.DEBUG):
            rospy.logdebug(m.msg)
        elif(m.level== Log.INFO):
            rospy.loginfo(m.msg)
        elif(m.level== Log.WARN):
            rospy.logwarn(m.msg)
        elif(m.level== Log.ERROR):
            rospy.logerr(m.msg)
        elif(m.level==Log.FATAL):
            rospy.logfatal(m.msg)
        
    def send(self, topic, msg):
        """ Send a message on a particular topic to the device. """
        with self.mutex:
            length = len(msg)
            checksum = 255 - ( ((topic&255) + (topic>>8) + (length&255) + (length>>8) + sum([ord(x) for x in msg]))%256 )
            data = '\xff\xff'+ chr(topic&255) + chr(topic>>8) + chr(length&255) + chr(length>>8)
            data = data + msg + chr(checksum)
            self.port.write(data)
