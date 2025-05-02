#!/usr/bin/env python3
import rospy
import numpy as np # type: ignore
from asclinic_pkg.msg import PoseCovar
import csv
from datetime import datetime
from std_msgs.msg import String

# Variance traces for logging
last_odom_var = float('nan')
last_aruco_var = float('nan')

# Global state and covariance
x_est = np.zeros((3, 1))  # [x, y, theta]
P_est = np.eye(3) * 0.01

driving_direction = "FORWARD"

# Time of last accepted ArUco update
last_aruco_accept_time = 0.0

# Process noise covariance
Q = np.diag([0.05, 5, 0.05])  # tune as needed

# Measurement noise covariance
R = np.diag([0.5, 0.5, 0.2])  # tune as needed

pose_pub = None

# CSV logging: unique file per run
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f'/home/asc/KF_testing_2/KF_testing_{timestamp}.csv'
csv_header_written = False

def log_to_csv(source, x, y, phi, xvar, yvar, phivar):
    global csv_filename
    global csv_header_written
    if not csv_header_written:
        with open(csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'time',
                'source',
                'x',
                'y',
                'phi',
                'xvar',
                'yvar',
                'phivar'
            ])
        csv_header_written = True
    try:
        with open(csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                rospy.Time.now().to_sec(),
                source,
                x,
                y,
                phi,
                xvar,
                yvar,
                phivar
            ])
    except Exception as e:
        rospy.logwarn(f"Failed to write to CSV in log_to_csv: {e}")

aruco_accepted = False

def normalize_angle(angle):
    # normalize radians to [-pi, pi)
    return (angle + np.pi) % (2 * np.pi) - np.pi

def driving_state_callback(msg):
    global driving_direction
    driving_direction = msg.data.upper()  # make it case-insensitive

def odom_callback(msg):
    global x_est, P_est, Q


    # Extract velocities and dt
    dx = msg.x
    dy = msg.y  # usually zero for differential drive
    # convert delta angle from degrees to radians
    dphi = np.deg2rad(msg.phi)

    theta = x_est[2, 0]

    # State prediction using motion model
    x_pred = np.zeros((3,1))

    #dx_world = np.cos(theta)*dx - np.sin(theta)*dy
    #dy_world = np.sin(theta)*dx + np.cos(theta)*dy
    x_pred[0,0] = x_est[0,0] + dx #dx_world
    x_pred[1,0] = x_est[1,0] + dy #dy_world
    x_pred[2,0] = normalize_angle(theta + dphi)

    # Jacobian F (motion model linearized)
    F = np.array([
        [1, 0, -dy],
        [0, 1,  dx],
        [0, 0, 1]
    ])

    # Covariance prediction
    P_pred = F.dot(P_est).dot(F.T) + Q

    x_est = x_pred
    P_est = P_pred

    # Log odometry-based estimate
    log_to_csv(
        'odom',
        float(x_est[0,0]),
        float(x_est[1,0]),
        float(np.rad2deg(x_est[2,0])),
        float(P_est[0,0]),
        float(P_est[1,1]),
        float(P_est[2,2])
    )

    publish_pose()

def aruco_callback(msg):
    global x_est, P_est, last_aruco_accept_time, aruco_accepted

    # convert orientation measurement from degrees to radians
    z = np.array([[msg.x], [msg.y], [np.deg2rad(msg.phi)]])
    # Construct measurement covariance matrix from message fields
    R_meas = np.array([
        [msg.xvar,   msg.xycovar, msg.xphicovar],
        [msg.xycovar, msg.yvar,   msg.yphicovar],
        [msg.xphicovar, msg.yphicovar, msg.phivar]
    ])
    # Measurement model (direct observation)
    H = np.eye(3)
    # Innovation (measurement residual)
    y = z - H.dot(x_est)
    y[2,0] = normalize_angle(y[2,0])
    # Innovation covariance
    S = H.dot(P_est).dot(H.T) + R_meas
    # Mahalanobis distance gating for [x,y,phi] at 95%
    current_time = rospy.Time.now().to_sec()
    # full residual (3×1) and covariance S (3×3)
    d2 = float(y.T.dot(np.linalg.inv(S)).dot(y))
    gamma = 7.82  # chi²(3 DOF, 95%) ≈ 7.815
    
    rospy.loginfo(f"residual  x={y[0,0]:.1f} mm, y={y[1,0]:.1f} mm, φ={y[2,0]:.1f} radians")
    rospy.loginfo(f"diag(S)  = [{S[0,0]:.3f}, {S[1,1]:.3f}, {S[2,2]:.3f}]  (units²)")

    if d2 <= gamma or (current_time - last_aruco_accept_time) >= 1.0:
        # accept this measurement
        K = P_est.dot(H.T).dot(np.linalg.inv(S))
        x_est = x_est + K.dot(y)
        x_est[2, 0] = normalize_angle(x_est[2, 0])
        P_est = (np.eye(3) - K.dot(H)).dot(P_est)
        last_aruco_accept_time = current_time
        aruco_accepted = True

        # Log aruco-based estimate
        log_to_csv(
            'aruco',
            float(x_est[0,0]),
            float(x_est[1,0]),
            float(np.rad2deg(x_est[2,0])),
            float(P_est[0,0]),
            float(P_est[1,1]),
            float(P_est[2,2])
        )
    else:
        rospy.logwarn(f"Aruco measurement rejected (d²={d2:.2f} > {gamma})")
        aruco_accepted = False
    # publish after gating
    publish_pose()

def publish_pose():
    global x_est, pose_pub
    global csv_header_written, csv_filename
    global last_odom_var, last_aruco_var
    global aruco_accepted

    pose_msg = PoseCovar()
    pose_msg.x = float(x_est[0,0])
    pose_msg.y = float(x_est[1,0])
    # convert internal radians back to degrees for publishing
    pose_msg.phi = float(np.rad2deg(x_est[2,0]))
    pose_msg.xvar = float(P_est[0,0])
    pose_msg.yvar = float(P_est[1,1])
    pose_msg.phivar = float(P_est[2,2])
    pose_msg.xycovar = float(P_est[0,1])
    pose_msg.xphicovar = float(P_est[0,2])
    pose_msg.yphicovar = float(P_est[1,2])

    #rospy.loginfo(f"Pose Estimate: x={pose_msg.x:.2f}, y={pose_msg.y:.2f}, phi={pose_msg.phi:.2f}°")
    pose_pub.publish(pose_msg)

def main():
    global pose_pub
    global csv_header_written
    # Create CSV file and write header
    # with open(csv_filename, 'w', newline='') as csvfile:
    #     writer = csv.writer(csvfile)
    #     writer.writerow([
    #         'time',
    #         'source',
    #         'x',
    #         'y',
    #         'phi',
    #         'xvar',
    #         'yvar',
    #         'phivar'
    #     ])
    rospy.init_node('ekf_fusion_node')

    pose_pub = rospy.Publisher('/pose_estimate_fused', PoseCovar, queue_size=10)
    rospy.Subscriber('/Pose', PoseCovar, odom_callback)
    rospy.Subscriber('/aruco_pose', PoseCovar, aruco_callback)
    rospy.Subscriber('/driving_state', String, driving_state_callback)

    rospy.spin()

if __name__ == '__main__':
    main()