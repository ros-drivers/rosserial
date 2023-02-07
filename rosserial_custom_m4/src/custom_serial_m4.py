#!/usr/bin/env python3
import rospy
# listener
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
# talker
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int8

odom_simple = PoseStamped()  # as given by camera_link
# odom_simple = Pose()  # as given by camera_link
def odom_cb(odom_full):
    odom_simple.header = odom_full.header
    odom_simple.pose = odom_full.pose.pose
    # odom_simple.position = odom_full.pose.pose.position
    # odom_simple.orientation = odom_full.pose.pose.orientation


status_simple = Int8()
def status_cb(status_full):
    if len(status_full.status_list) > 0:
        status_simple.data = status_full.status_list[-1].status


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/rtabmap/odom", Odometry, odom_cb)
    rospy.Subscriber("/move_base/status", GoalStatusArray, status_cb)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


def talker():
    odom = rospy.Publisher('/rtabmap/odom_simple', PoseStamped, queue_size=10)
    status = rospy.Publisher('/move_base/status_simple', Int8, queue_size=10)
    
    # rospy.init_node('talker', anonymous=True)

    while not rospy.is_shutdown():        
        odom.publish(odom_simple)
        status.publish(status_simple)

        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('m4_relay_node', anonymous=True)
        rate = rospy.Rate(50) # Hz, loop frequency on arduino
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass


