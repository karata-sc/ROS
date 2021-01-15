#!/usr/bin/env python2
import rospy
from std_msgs.msg import Int32

def cb(message):
    rospy.loginfo(message.data)

if __name__ == '__main__':
    rospy.init_node('enc_sub')
    sub = rospy.Subscriber('enc_data', Int32, cb)
    rospy.spin()
