#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import csv
from asclinic_pkg.msg import FiducialMarkerArray

measurement_limit = 50

# Set the current test conditions:
# Choose one of the following distances (in metres): 1, 4, or 6
CURRENT_DISTANCE = 4
# Choose the expected pitch angle (in degrees) from the options: 0, 45, 75
CURRENT_EXPECTED_PITCH = 45

def get_true_position(distance, pitch):
    """
    Returns the true measured position for a marker that is directly ahead of the robot.
    Since the marker is directly ahead, its expected front position (in the x-z plane) is:
        [0, 0, distance]
    The function also returns the expected pitch angle.
    """
    return {'front': [0, 0, distance], 'expected_pitch': pitch}

# Global list to store measurement data
data_collection = []

def callback(data):
    # Check if we have already collected the required number of measurements
    if len(data_collection) >= measurement_limit:
        return

    if data.num_markers > 0:
        rospy.loginfo("--- Received ArUco Marker ---")
        for marker in data.markers:
            if len(data_collection) >= measurement_limit:
                rospy.signal_shutdown("Collected 50 measurements")
                return

            marker_id = marker.id
            rvec = marker.rvec
            tvec = marker.tvec

            # Convert rotation vector to rotation matrix
            R, _ = cv2.Rodrigues(np.array(rvec))
            R_inv = R.T
            tvec_np = np.array(tvec).reshape(3, 1)
            # Calculate the camera (robot) position in the marker frame
            camera_position = -R_inv @ tvec_np

            pitch = np.arcsin(-R_inv[2, 0])
            pitch_degree = np.degrees(pitch)

            # Compute the front position by transforming a fixed offset in the camera frame.
            # Here the camera is treated as an inertial frame with a -150 mm offset along its local z-axis.
            t_offset_cam = np.array([[0], [0], [-0.15]])
            offset_marker = R_inv @ t_offset_cam
            # Since the robot's front is closer to the marker than the camera, subtract the transformed offset.
            front_position = camera_position - offset_marker

            measurement = {
                'timestamp': rospy.get_time(),
                'distance': CURRENT_DISTANCE,
                'expected_pitch': CURRENT_EXPECTED_PITCH,
                'marker_id': marker_id,
                'pitch_degree': pitch_degree,
                'camera_x': camera_position[0][0],
                'camera_y': camera_position[1][0],
                'camera_z': camera_position[2][0],
                'front_x': front_position[0][0],
                'front_y': front_position[1][0],
                'front_z': front_position[2][0]
            }
            data_collection.append(measurement)

            rospy.loginfo("Measurement collected: %s", measurement)
            rospy.loginfo("---------------------------------------------------")

            if len(data_collection) >= measurement_limit:
                rospy.signal_shutdown("Collected 50 measurements")
                return

def save_data():
    filename = f"stationary_test_data_distance_{CURRENT_DISTANCE}_pitch_{CURRENT_EXPECTED_PITCH}.csv"
    fieldnames = ['timestamp', 'distance', 'expected_pitch', 'marker_id', 'pitch_degree',
                  'camera_x', 'camera_y', 'camera_z', 'front_x', 'front_y', 'front_z']
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for d in data_collection:
            writer.writerow(d)
    rospy.loginfo("Data saved to %s", filename)

def listener():
    rospy.init_node('aruco_positioning', anonymous=True)
    rospy.Subscriber("/asc/aruco_detections", FiducialMarkerArray, callback)
    rospy.spin()
    save_data()

if __name__ == '__main__':
    true_pos = get_true_position(CURRENT_DISTANCE, CURRENT_EXPECTED_PITCH)
    rospy.loginfo("Expected true position: %s", true_pos)
    listener()