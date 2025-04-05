#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np # type: ignore
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
            # Calculate the robot position and pose in the world frame
            camera_position = -R_inv @ tvec_np
            yaw = np.arctan2(R_inv[1, 0], R_inv[0, 0])
            
            # Compute the front of the robot's position by transforming a fixed offset in the camera frame
            # Here, the camera is treated as a separate inertial frame with a -150 mm offset along the z-axis (pointing out of the camera)
            # The offset vector in the camera frame (in meters) is:
            t_offset_cam = np.array([[0], [0], [-0.15]])
            
            # Transform the offset from the camera frame to the marker frame using the rotation matrix R_inv
            offset_marker = R_inv @ t_offset_cam
            
            # The front position in the marker frame is the camera position plus the transformed offset
            front_position = camera_position - offset_marker

            yaw_degree = np.degrees(yaw)

            # Log the information
            rospy.loginfo("Marker ID: %d", marker_id)
            rospy.loginfo("Camera Position in Marker Frame: [%f, %f, %f]", camera_position[0][0], camera_position[1][0], camera_position[2][0])
            rospy.loginfo("Robot Yaw in Marker Frame: %f", yaw_degree)
            rospy.loginfo("Front of Robot Position in Marker Frame: [%f, %f, %f]", front_position[0][0], front_position[1][0], front_position[2][0])
            rospy.loginfo("---------------------------------------------------")

def listener():
    rospy.init_node('aruco_positioning', anonymous=True)
    rospy.Subscriber("/asc/aruco_detections", FiducialMarkerArray, callback)
    rospy.spin()

if __name__ == '__main__':
        listener()