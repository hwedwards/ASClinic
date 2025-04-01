#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from asclinic_pkg.msg import FiducialMarkerArray

def callback(data):
    if data.num_markers >0:
        rospy.loginfo("--- Received ArUco Marker ---")
        for marker in data.markers:
            marker_id = marker.id
            rvec = marker.rvec
            tvec = marker.tvec

            # Convert rotation vector to rotation matrix
            R, _ = cv2.Rodrigues(np.array(rvec))

            # Calculate inverse of rotation matrix
            R_inv = R.T 
            tvec_np = np.array(tvec).reshape(3, 1)
            # Calculate the robot position and posein the world frame
            robot_position = -R_inv @ tvec_np
            yaw = np.arctan2(R_inv[1, 0], R_inv[0, 0])
            yaw_degree = np.degrees(yaw)

            # Log the information
            rospy.loginfo("Marker ID: %d", marker_id)
            rospy.loginfo("Robot Position in Marker Frame: [%f, %f, %f]", robot_position[0][0], robot_position[1][0], robot_position[2][0])
            rospy.loginfo("Robot Yaw in Marker Frame: %f", yaw_degree)
            rospy.loginfo("---------------------------------------------------")

def listener():
    rospy.init_node('aruco_positioning', anonymous=True)
    rospy.Subscriber("/asc/aruco_detections", FiducialMarkerArray, callback)
    rospy.spin()

if __name__ == '__main__':
        listener()