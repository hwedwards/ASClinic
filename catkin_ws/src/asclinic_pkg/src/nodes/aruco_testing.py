#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import csv
from asclinic_pkg.msg import FiducialMarkerArray

measurement_limit = 50


def get_true_position(position_id):
    true_positions = {
        1: {'front': [0, 0, 1]},
        2: {'front': [-0.5, 0, 1]},
        3: {'front': [0.5, 0, 1]},
        4: {'front': [0, 0, 2]},
        5: {'front': [-0.5, 0, 2]},
        6: {'front': [0.5, 0, 2]},
        7: {'front': [0, 0, 3]},
        8: {'front': [-0.5, 0, 3]},
        9: {'front': [0.5, 0, 3]},
        10: {'front': [0, 0, 4]},
        11: {'front': [-0.5, 0, 4]},
        12: {'front': [0.5, 0, 4]},
        13: {'front': [0, 0, 7]},
        14: {'front': [0.8, 0, 7]},
        15: {'front': [-0.8, 0, 7]},
        16: {'front': [0, 0, 10]},
        17: {'front': [1.2, 0, 10]},
        18: {'front': [-1, 0, 10]},
        19: {'front': [0, 0, 14]},
        20: {'front': [0, 0, 5]},
        21: {'front': [-0.5, 0, 5]},
        22: {'front': [0.5, 0, 5]},
        23: {'front': [0, 0, 9.8]},
        24: {'front': [-1, 0, 9.8]},
        25: {'front': [1, 0, 9.8]}
    }
    return true_positions.get(position_id, None)

# Set the current position id for this stationary test
CURRENT_POSITION_ID = 2

# Global list to store measurement data
data_collection = []

def callback(data):
    # Check if we have already collected the required number of measurements
    if len(data_collection) >= measurement_limit:
        return

    if data.num_markers > 0:
        rospy.loginfo("--- Received ArUco Marker ---")
        for marker in data.markers:
            # If limit is reached inside loop, break
            if len(data_collection) >= measurement_limit:
                rospy.signal_shutdown("Collected 100 measurements")
                return

            marker_id = marker.id
            rvec = marker.rvec
            tvec = marker.tvec

            # Convert rotation vector to rotation matrix
            R, _ = cv2.Rodrigues(np.array(rvec))

            # Calculate inverse of rotation matrix
            R_inv = R.T
            tvec_np = np.array(tvec).reshape(3, 1)
            # Calculate the camera (robot) position in the marker frame
            camera_position = -R_inv @ tvec_np

            # Compute the front position by transforming a fixed offset in the camera frame
            # Camera is treated as an inertial frame with a -150 mm offset along its local z-axis.
            t_offset_cam = np.array([[0], [0], [-0.15]])
            # Transform the offset to the marker frame
            offset_marker = R_inv @ t_offset_cam
            # Since the robot front is closer to the marker, subtract the transformed offset
            front_position = camera_position - offset_marker

            # Create a measurement dictionary
            measurement = {
                'timestamp': rospy.get_time(),
                'position_id': CURRENT_POSITION_ID,
                'marker_id': marker_id,
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
                rospy.signal_shutdown("Collected 100 measurements")
                return

def save_data():
    filename = f"stationary_test_data_point_{CURRENT_POSITION_ID}.csv"    
    fieldnames = ['timestamp', 'position_id', 'marker_id', 'camera_x', 'camera_y', 'camera_z',
                    'front_x', 'front_y', 'front_z']
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
    listener()