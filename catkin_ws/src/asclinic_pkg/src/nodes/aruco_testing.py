#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import csv
import math
from asclinic_pkg.msg import FiducialMarkerArray


# World coordinates of ArUco markers (x, y)
marker_world_positions = {
    1: (1.973, -0.600),
    2: (2.033, -0.600),
    3: (3.952, 1.200),
    4: (4.012, 1.200),
    5: (5.953, -0.600),
    6: (6.013, -0.600),
    7: (7.926, 1.200),
    8: (7.986, 1.200),
    9: (9.913, -0.600),
    10: (9.973, -0.600),
    11: (11.900, 0.000),
    12: (9.943, 1.500),
    13: (0.030, -0.600),
    14: (0.030, 1.200),
}


# Set the current position id for this stationary test
CURRENT_POSITION_ID = 10


def get_true_pose(position_id):
    true_poses = {
        1: (0, 0, 0), #1
        2: (2, 0, 0), #2
        3: (2, 0, 180), #2
        4: (4, 0, 0), #3
        5: (4, 0, 180), #3
        6: (6, 0, 0), #4
        7: (6, 0, 180), #4
        8: (8, 0, 0), #5
        9: (8, 0, 180), #5
        10: (10, 0, 0), #6
        11: (10, 0, 180) #6
    }
    return true_poses.get(position_id, None)

measurement_limit = 200

marker_positions = {
    1: (1.973, -0.600, 180),
    2: (2.033, -0.600,   0),
    3: (3.952,  1.200, 180),
    4: (4.012,  1.200,   0),
    5: (5.953, -0.600, 180),
    6: (6.013, -0.600,   0),
    7: (7.926,  1.200, 180),
    8: (7.986,  1.200,   0),
    9: (9.913, -0.600, 180),
   10: (9.973, -0.600,   0),
   11: (11.900, 0.000, 180),
   12: (9.943,  1.500, -90),
   13: (0.030, -0.600,   0),
   14: (0.030,  1.200,   0),
}

def get_marker_pose(marker_id):
    """
    Return the (x, y, phi) world pose for a given ArUco marker ID,
    with phi normalized to [0, 360). Logs a warning and returns None if unknown.
    """
    m_pose = marker_positions.get(marker_id)
    if m_pose is None:
        rospy.logwarn(f"Unknown marker ID: {marker_id}")
        return None
    x, y, phi = m_pose
    # Normalize heading to [0, 360)
    phi = phi % 360
    return x, y, phi




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

            # We only care about pitch angle as it is about the y-axis
            pitch = np.arcsin(-R_inv[2, 0])
            pitch_degree = np.degrees(pitch)
            # Compute the front position by transforming a fixed offset in the camera frame
            # Camera is treated as an inertial frame with a -150 mm offset along its local z-axis.
            t_offset_cam = np.array([[0], [0], [-0.15]])
            # Transform the offset to the marker frame
            offset_marker = R_inv @ t_offset_cam
            # Since the robot front is closer to the marker, subtract the transformed offset
            front_position = camera_position - offset_marker

            # Compute front position in world frame
            m_pose = get_marker_pose(marker_id)
            if m_pose is not None:
                x_world, y_world, phi_deg = m_pose
                # Normalize marker heading to [0, 360)
                phi_deg = phi_deg % 360
                # Convert marker heading to radians
                phi_rad = math.radians(phi_deg)
                # Relative front position in marker frame (ignoring vertical axis)
                # Marker frame Z axis (depth) = forward; marker frame X axis (lateral) = sideways
                forward_rel = front_position[2][0]  # forward offset (marker Z)
                lateral_rel = front_position[0][0]  # lateral offset (marker X)
                # Rotate and translate into world frame (X forward, Y lateral)
                x_front_world = x_world + forward_rel*math.cos(phi_rad) - lateral_rel*math.sin(phi_rad)
                y_front_world = y_world + forward_rel*math.sin(phi_rad) + lateral_rel*math.cos(phi_rad)
                # Compute camera yaw around vertical axis (Y) from rotation matrix:
                # Camera forward vector in marker frame is R_inv[:,2]; extract ground-plane components:
                yaw_rad = math.atan2(R_inv[0,2], R_inv[2,2])
                yaw_deg = math.degrees(yaw_rad)
                # Normalize camera yaw to [0, 360)
                yaw_deg = yaw_deg % 360
                phi_front = (phi_deg - yaw_deg) % 360
            
            # Get the true pose for the current position ID
            true_pose = get_true_pose(CURRENT_POSITION_ID)
            # Create a measurement dictionary
            measurement = {
                'timestamp': rospy.get_time(),
                'position_id': CURRENT_POSITION_ID,
                'marker_id': marker_id,
                'true_x': true_pose[0],
                'true_y': true_pose[1],
                'true_phi': true_pose[2],
                'marker_x': front_position[2][0],
                'marker_y': front_position[0][0],
                'marker_phi': pitch_degree, 
                'est_x': x_front_world,
                'est_y': y_front_world,
                'est_phi': phi_front,
            }
            data_collection.append(measurement)

            rospy.loginfo("Measurement collected: %s", measurement)
            rospy.loginfo("---------------------------------------------------")

            if len(data_collection) >= measurement_limit:
                rospy.signal_shutdown("Collected 200 measurements")
                return

def save_data():
    filename = "aruco_testing.csv"
    # Check if file exists to decide whether to write header
    file_exists = os.path.isfile(filename)
    if not data_collection:
        rospy.logwarn("No measurements collected; CSV not created.")
        return
    # Use measurement dictionary keys as CSV columns
    fieldnames = list(data_collection[0].keys())
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        for d in data_collection:
            # Ensure each row matches the header order
            row = {key: d.get(key, "") for key in fieldnames}
            writer.writerow(row)
    rospy.loginfo("Data saved to %s", filename)

def listener():
    rospy.init_node('aruco_positioning', anonymous=True)
    rospy.Subscriber("/asc/aruco_detections", FiducialMarkerArray, callback)
    rospy.spin()
    save_data()

if __name__ == '__main__':
    # 1. Load measurement CSV
    filename = "aruco_testing.csv"
    import pandas as pd
    df = pd.read_csv(filename)

    # 2. Map marker world positions
    df['marker_world_x'] = df['marker_id'].map(lambda m: marker_world_positions[m][0])
    df['marker_world_y'] = df['marker_id'].map(lambda m: marker_world_positions[m][1])

    # 3. Compute distance from robot to marker
    df['distance'] = np.sqrt((df['true_x'] - df['marker_world_x'])**2 +
                             (df['true_y'] - df['marker_world_y'])**2)

    # 4. Compute error terms (independent vs dependent variables)
    df['err_x'] = df['est_x'] - df['true_x']
    df['err_y'] = df['est_y'] - df['true_y']
    # Orientation error wrapped to [-180, 180], then converted to radians
    dphi = (df['est_phi'] - df['true_phi'] + 180) % 360 - 180
    df['err_phi'] = np.deg2rad(dphi)

    # 5. Bin by distance and compute covariance matrices
    bin_edges = np.arange(0.0, df['distance'].max() + 0.5, 0.5)
    df['bin'] = pd.cut(df['distance'], bin_edges, labels=bin_edges[:-1] + 0.25)
    cov_by_bin = {}
    for d, grp in df.groupby('bin'):
        errors = grp[['err_x', 'err_y', 'err_phi']].dropna().to_numpy().T
        if errors.shape[1] >= 2:
            cov_by_bin[float(d)] = np.cov(errors, bias=False)

    # 6. Print covariance per distance bin
    for d in sorted(cov_by_bin.keys()):
        print(f"Distance bin {d:.2f} m: covariance matrix:")
        print(cov_by_bin[d])
        print()