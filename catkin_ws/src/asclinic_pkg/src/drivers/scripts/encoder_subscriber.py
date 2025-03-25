#!/usr/bin/env python
import rospy
from asclinic_pkg.msg import LeftRightInt32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "From the encoders, L: %d & Right: %d", data.left, data.right)

def listener():
    rospy.init_node('encoder_subscriber', anonymous=True)
    rospy.Subscriber("/asc/encoder_counts", LeftRightInt32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()