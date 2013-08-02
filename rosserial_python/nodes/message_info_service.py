#!/usr/bin/env python

import rospy
from rosserial_msgs import RequestMessageInfo
from rosserial_python import load_message


def cb(req):  

if __name__=="__main__":
    rospy.init_node("message_info_service")
    rospy.loginfo("rosserial message_info_service node")
    rospy.Server("message_info", RequestMessageInfo, cb)
    rospy.spin()
