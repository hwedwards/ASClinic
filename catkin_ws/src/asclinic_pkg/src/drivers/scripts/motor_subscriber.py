#!/usr/bin/env python
import rospy
from asclinic_pkg.msg import LeftRightFloat32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard: L: %f & Right: %f", data.left, data.right)

def listener():
    rospy.init_node('motor_subscriber', anonymous=True)
    rospy.Subscriber("/asc/set_motor_duty_cycle", LeftRightFloat32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()