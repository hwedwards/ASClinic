#!/usr/bin/env python
import rospy
from asclinic_pkg.msg import ServoPulseWidth
import time

def main():
    rospy.init_node("servo_channel_scanner", anonymous=True)
    pub = rospy.Publisher("/asc/set_servo_pulse_width", ServoPulseWidth, queue_size=10)

    rospy.loginfo("Starting scan of PCA9685 servo channels (0-15)")

    # Define test pulse width (safe default for most servos)
    test_pulse_width = 1100  # microseconds

    # Loop through all channels
    for channel in range(16):
        msg = ServoPulseWidth()
        msg.channel = channel
        msg.pulse_width_in_microseconds = test_pulse_width

        rospy.loginfo("Sending test pulse to channel %d", channel)
        pub.publish(msg)
        rospy.sleep(1.0)  # Give servo time to respond

    rospy.loginfo("Servo channel scan complete.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
