#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt32

def timer_callback(event):
    # Publish a message with any UInt32 value (not used by subscriber)
    pub.publish(UInt32(1))

if __name__ == '__main__':
    rospy.init_node('image_saver_trigger')
    pub = rospy.Publisher('/asc/request_save_image', UInt32, queue_size=10)

    # Set timer to trigger at 3 Hz
    rospy.Timer(rospy.Duration(1.0 / 0.3), timer_callback)

    rospy.loginfo("Image saver trigger running at 0.5 Hz...")
    rospy.spin()