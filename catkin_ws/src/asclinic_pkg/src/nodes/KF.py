#!/usr/bin/env python3
import rospy
import numpy as np # type: ignore
from asclinic_pkg.msg import PoseCovar
import csv
from datetime import datetime
from std_msgs.msg import String

 # Odometry position variables
x_odom = 0.0
y_odom = 0.0

# Global state and covariance
x_est = np.zeros((3, 1))  # [x, y, theta in radians]
# manually setting the initial pose to something other than the origin
#x_est[0] = 5000
#x_est[1] = 0
#x_est[2] = np.pi # initial heading in degrees
P_est = np.eye(3) * 0.01
#x_est[2,0] = np.pi/4 # initial heading

# Process noise covariance
Q = np.diag([100, 5000, 100])  # tune as needed

# Measurement noise covariance (increase R to decrease influence of ArUco and increase low pass characteristics of KF)
R = np.diag([10, 10, 1])  # tune as needed

pose_pub = None

# CSV logging: unique file per run
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f'/home/asc/KF_testing_2/KF_testing_{timestamp}.csv'
csv_header_written = False
# Innovations logging
innovations_csv_filename = f'/home/asc/KF_testing_2/innovations_{timestamp}.csv'
innovations_header_written = False

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

def log_innovation_to_csv(y):
    global innovations_csv_filename
    global innovations_header_written
    if not innovations_header_written:
        with open(innovations_csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time','res_x','res_y','res_phi'])
        innovations_header_written = True
    try:
        with open(innovations_csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                rospy.Time.now().to_sec(),
                float(y[0,0]),
                float(y[1,0]),
                float(np.rad2deg(y[2,0]))
            ])
    except Exception as e:
        rospy.logwarn(f"Failed to write to innovations CSV in log_innovation_to_csv: {e}")

aruco_accepted = False

def normalize_angle(angle):
    # normalize radians to [-pi, pi)
    return (angle + np.pi) % (2 * np.pi) - np.pi

def driving_state_callback(msg):
    global driving_direction
    driving_direction = msg.data.upper()  # make it case-insensitive

def odom_callback(msg):
    global x_est, P_est, Q
    global x_odom, y_odom

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

    x_odom = x_odom+dx
    y_odom = y_odom+dy

    # Log odometry-based estimate
    log_to_csv(
        'odom',
        float(x_odom),
        float(y_odom),
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
    distance = msg.dist*1000  # convert to mm

    eig = np.linalg.eigvals(R_meas)
    if np.any(eig <= 0):
        rospy.logwarn(f"Bad R_meas (eig={eig}); replacing with nominal diag.")
        R_meas = np.diag([25.0, 25.0, np.deg2rad(5)**2])

    # Measurement model (direct observation)
    H = np.eye(3)
    # Innovation (measurement residual)
    y = z - H.dot(x_est)
    y[2,0] = normalize_angle(y[2,0])
    # log the raw innovation (pre-gating)
    log_innovation_to_csv(y)
    # Innovation covariance
    S = H.dot(P_est).dot(H.T) + R_meas

    # Euclidean distance gate
    if distance > 3000:  
        rospy.logwarn(f"ArUco rejected (Distance={distance:.1f})")
        return
    else: 
        rospy.logwarn(f"ArUco ACCEPTED (Distance={distance:.1f})")
        log_to_csv(
        'PURE_ARUCO',
        float(msg.x),
        float(msg.y),
        float(msg.phi),
        float(R_meas[0,0]),
        float(R_meas[1,1]),
        float(R_meas[2,2])
    )

    # Regularise S if it is near-singular
    cond = np.linalg.cond(S)
    if cond > 1e8 or np.any(np.isnan(S)):
        rospy.logwarn(f"S ill-conditioned (cond={cond:.2e}); adding jitter.")
        S += np.eye(3) * 1e-2
    K = P_est.dot(H.T).dot(np.linalg.inv(S))
    
    """rospy.loginfo(f"residual  x={y[0,0]:.1f} mm, y={y[1,0]:.1f} mm, φ={y[2,0]:.1f} radians")
    rospy.loginfo(f"diag(S)  = [{S[0,0]:.3f}, {S[1,1]:.3f}, {S[2,2]:.3f}]  (units²)")"""
    
    log_to_csv(
        'KF_K',
        float(K[0,0]),
        float(K[1,1]),
        float(K[2,2]),
        0,
        0,
        0
    )

    # Compute state correction and saturate based on delta thresholds
    dx = K.dot(y)

    # Saturation thresholds
    max_dx = 28 # 100       # mm
    max_dy = 28 # 100       # mm # 28 in both dimensions limits the maximum jump to 4 cm per timestep which is 40 cm per second
    max_dphi = np.deg2rad(10)  # radians

    dx[0,0] = np.clip(dx[0,0], -max_dx, max_dx)
    dx[1,0] = np.clip(dx[1,0], -max_dy, max_dy)
    dx[2,0] = np.clip(dx[2,0], -max_dphi, max_dphi)

    # Apply saturated update
    x_est = x_est + dx
    x_est[2,0] = normalize_angle(x_est[2,0])
    # Joseph-form covariance update to maintain positive semidefiniteness
    I = np.eye(3)
    P_est = (I - K.dot(H)).dot(P_est).dot((I - K.dot(H)).T) + K.dot(R_meas).dot(K.T)
    # enforce symmetry to remove numerical asymmetry
    P_est = 0.5 * (P_est + P_est.T)
    aruco_accepted = True

    # publish after gating
    publish_pose()

def publish_pose():
    global x_est, pose_pub
    global csv_header_written, csv_filename
    global last_odom_var, last_aruco_var
    global aruco_accepted
    global K

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

    # Log KF estimate
    log_to_csv(
        'KF',
        float(x_est[0,0]),
        float(x_est[1,0]),
        float(np.rad2deg(x_est[2,0])),
        float(P_est[0,0]),
        float(P_est[1,1]),
        float(P_est[2,2])
    )

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