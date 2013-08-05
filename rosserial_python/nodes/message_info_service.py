#!/usr/bin/env python

import rospy
from rosserial_msgs.srv import RequestMessageInfo
from rosserial_python import load_message


class MessageInfoService(object):
  def __init__(self):
    rospy.init_node("message_info_service")
    rospy.loginfo("rosserial message_info_service node")
    self.service = rospy.Service("message_info", RequestMessageInfo, self._cb)
    self.cache = {}
  
  def _cb(self, req):
    package_message = tuple(req.type.split("/"))
    if not self.cache.has_key(package_message):
      rospy.loginfo("Loading module to return info on %s/%s." % package_message)
      msg = load_message(*package_message)
      self.cache[package_message] = (msg._md5sum, msg._full_text)
    else:
      rospy.loginfo("Returning info from cache on %s/%s." % package_message)

    return self.cache[package_message]

  def spin(self):
    rospy.spin()

if __name__=="__main__":
  MessageInfoService().spin()
