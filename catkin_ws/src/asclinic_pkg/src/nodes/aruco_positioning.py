#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv
from datetime import datetime

csv_filename = f"/home/asc/KF_testing_2/aruco_positions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_header_written = False


import rospy
import numpy as np # type: ignore
import cv2
import math
# Scaling factors
MM2 = 1e6  # (1000 mm/m)^2 for variances in mm^2
MM = 1000.0  # for covariances mixing mm with radians or degrees
RAD2DEG = 180.0 / math.pi
RAD2DEG2 = RAD2DEG ** 2
# Marker frame axes: X is lateral (sideways), Y is vertical (ignored), Z is depth (forward).
# World frame axes: X is longitudinal (forward), Y is lateral (sideways).
# Therefore marker frame Z maps to world X, and marker frame X maps to world Y.




from asclinic_pkg.msg import FiducialMarkerArray
from asclinic_pkg.msg import PoseCovar
pose_pub = None

# Counter to throttle logging: only log every 10 callbacks
log_counter = 1

# Mapping from ArUco marker ID to its measured world-frame position (x, y) and heading phi (degrees)
marker_positions = {
    12: (-0.8, -3.0, 90),
    13: (1.0, -4.5, 90),
    15: (1.5, -3.0, 180)
   }
"""    1: (1.973, -0.600, 180),
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
   14: (0.030,  1.200,   0)"""


def get_marker_pose(marker_id):
    """
    Return the (x, y, phi) world pose for a given ArUco marker ID.
    If the ID is not in the table, logs a warning and returns None.
    """
    pose = marker_positions.get(marker_id)
    if pose is None:
        rospy.logwarn(f"Unknown marker ID: {marker_id}")
    return pose

def callback(data):
    if data.num_markers >0:
        global log_counter
        log_counter += 1
        should_log = (log_counter % 100000000000 == 0)
        if should_log:
            rospy.loginfo("--- Received ArUco Marker ---")
        for marker in data.markers:
            marker_id = marker.id
            timestamp = rospy.Time.now().to_sec()
        
            if should_log:
                rospy.loginfo("Marker ID: %d", marker_id)

            # Skip processing for markers not in dictionary
            pose = get_marker_pose(marker_id)
            if pose is None:
                continue

            rvec = marker.rvec
            tvec = marker.tvec

            # Convert rotation vector to rotation matrix
            R, _ = cv2.Rodrigues(np.array(rvec))

            # Calculate inverse of rotation matrix
            R_inv = R.T 
            tvec_np = np.array(tvec).reshape(3, 1)
            # Calculate the robot position and pose in the world frame
            camera_position = -R_inv @ tvec_np

            

            
            # We only care about pitch angle as it is about the y-axis
            pitch = np.arcsin(-R_inv[2, 0])
            pitch_degree = np.degrees(pitch)

            # Compute the front of the robot's position by transforming a fixed offset in the camera frame
            # Here, the camera is treated as a separate inertial frame with a -150 mm offset along the z-axis (pointing out of the camera)
            # The offset vector in the camera frame (in meters) is:
            t_offset_cam = np.array([[0], [0], [-0.15]])
            
            # Transform the offset from the camera frame to the marker frame using the rotation matrix R_inv
            offset_marker = R_inv @ t_offset_cam
            
            # The front position in the marker frame is the camera position plus the transformed offset
            front_position = camera_position - offset_marker

            # Log the information
            """rospy.loginfo("Robot Pitch in Marker Frame: %f", pitch_degree)
            rospy.loginfo("Front of Robot Position in Marker Frame: [%f, %f, %f]", front_position[0][0], front_position[1][0], front_position[2][0])
            """
            # Use pose obtained earlier
            x_world, y_world, phi_deg = pose
            # Convert marker heading to radians
            phi_rad = math.radians(phi_deg)
            # Relative front position in marker frame (ignoring vertical axis)
            # Marker frame Z axis (depth) = forward; marker frame X axis (lateral) = sideways
            forward_rel = front_position[2][0]  # forward offset (marker Z)
            lateral_rel = front_position[0][0]  # lateral offset (marker X)
            # Rotate and translate into world frame (X forward, Y lateral)
            x_front_world = x_world + forward_rel*math.cos(phi_rad) + lateral_rel*math.sin(phi_rad)
            y_front_world = y_world + forward_rel*math.sin(phi_rad) - lateral_rel*math.cos(phi_rad)
            # Compute camera yaw around vertical axis (Y) from rotation matrix:
            # Camera forward vector in marker frame is R_inv[:,2]; extract ground-plane components:
            yaw_rad = math.atan2(R_inv[0,2], R_inv[2,2])
            yaw_deg = math.degrees(yaw_rad)
            phi_front = phi_deg - yaw_deg
            # Log world pose of front of robot
            if should_log:
                rospy.loginfo("Front of Robot in World Frame: x=%.3f, y=%.3f, phi=%.1f°",
                              x_front_world, y_front_world, phi_front)
            # Compute distance from robot front to marker
            distance = math.hypot(lateral_rel, forward_rel)

            # Quadratic-fit covariance model coefficients (updated)
            c00 = (-7.906e-04,  9.596e-03, -1.704e-02)
            c01 = ( 8.108e-04, -9.677e-03,  1.783e-02)
            c02 = (-4.376e-05,  5.149e-04, -1.076e-03)
            c11 = (-5.360e-04,  1.491e-02, -3.416e-02)
            c12 = (-1.960e-04,  2.016e-03, -4.329e-03)
            c22 = (-1.185e-05,  1.536e-04,  1.299e-04)

            def quad_val(coefs, d):
                return coefs[0]*d**2 + coefs[1]*d + coefs[2]

            # Compute raw covariance entries
            r00 = quad_val(c00, distance)
            r11 = quad_val(c11, distance)
            # Override small-distance thresholds
            if distance < 3.885:
                r00 = 0.0001
            if distance < 4.247:
                r11 = 0.0001
            r01 = quad_val(c01, distance)
            r02 = quad_val(c02, distance)
            r12 = quad_val(c12, distance)
            r22 = quad_val(c22, distance)

            # Assemble full 3×3 covariance matrix with proper unit conversions 
            # X and Y swapped to account for change of aruco to world frame
            R = np.array([
                [r11 * MM2, r12 * MM2, r02 * MM],  # var_y, cov(y,x), cov(y,phi)
                [r12 * MM2, r00 * MM2, r01 * MM],  # cov(x,y), var_x, cov(x,phi)
                [r02 * MM, r01 * MM, r22]  # cov(phi,y), cov(phi,x), var_phi
            ])

            # ------------------------------------------------------------------
            # 1. Force symmetry
            R = 0.5*(R + R.T)

            # 2a. Zero the cross terms for now (keeps R diagonal-dominant)
            R[0,1] = R[1,0] = 0
            R[0,2] = R[2,0] = 0
            R[1,2] = R[2,1] = 0

            # 2b. Floor each variance to a realistic minimum (1 mm², (2°)² )
            R[0,0] = max(R[0,0], 1.0)
            R[1,1] = max(R[1,1], 1.0)
            R[2,2] = max(R[2,2], np.deg2rad(2)**2)

            # 3. Final eigen-clip to be absolutely safe
            eigvals, eigvecs = np.linalg.eigh(R)
            eigvals = np.maximum(eigvals, 1e-6)
            R = eigvecs @ np.diag(eigvals) @ eigvecs.T
            # ------------------------------------------------------------------

            # Log the covariance
            if should_log:
                rospy.loginfo("Covariance R(d=%.2f): %s", distance, R.tolist())
                rospy.loginfo("---------------------------------------------------")

    
            # Publish PoseCovar message
            msg = PoseCovar()
            msg.x = x_front_world*1000
            msg.y = y_front_world*1000
            msg.phi = phi_front

            # Extract covariances from R (swapped axes)
            msg.xvar = float(R[1, 1])
            msg.yvar = float(R[0, 0])
            msg.phivar = float(R[2, 2])
            msg.xycovar = float(R[0, 1])
            msg.xphicovar = float(R[1, 2])
            msg.yphicovar = float(R[0, 2])

            
            if not np.isfinite(distance) or distance > 10:
                msg.dist = 10
            else:   
                msg.dist = distance

            pose_pub.publish(msg)

            global csv_header_written
            with open(csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                if not csv_header_written:
                    writer.writerow(['timestamp', 'marker_id', 'x_mm', 'y_mm', 'phi_deg'])
                    csv_header_written = True
                writer.writerow([timestamp, marker_id, msg.x, msg.y, msg.phi])

def listener():
    global pose_pub
    rospy.init_node('aruco_positioning', anonymous=True)
    pose_pub = rospy.Publisher('aruco_pose', PoseCovar, queue_size=10)
    rospy.Subscriber("/asc/aruco_detections", FiducialMarkerArray, callback)
    rospy.spin()

if __name__ == '__main__':
        listener()