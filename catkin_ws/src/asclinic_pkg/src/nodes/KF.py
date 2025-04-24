#!/usr/bin/env python3
import rospy
import numpy as np
import csv
from asclinic_pkg.msg import PoseCovar

class KFNode:
    def __init__(self):
        # State vector [x, y, phi]
        self.x = np.zeros((3, 1))
        # State covariance
        self.P = np.eye(3) * 1.0
        # Process noise covariance (tune as needed)
        q_pos = 0.01
        q_ang = 0.01
        self.Q = np.diag([q_pos, q_pos, q_ang])

        # Publisher for fused pose+covariance
        self.fused_pub = rospy.Publisher('/pose_estimate_fused', PoseCovar, queue_size=10)
        rospy.Subscriber('/Pose', PoseCovar, self.odom_callback)
        rospy.Subscriber('/aruco_pose', PoseCovar, self.aruco_callback)

        self.initialized = False

        # Open single CSV for logging all data
        self.all_csv = open('/home/asc/ASClinic/all_data_log.csv', 'w', newline='')
        self.all_writer = csv.writer(self.all_csv)
        self.all_writer.writerow([
            'type','time','x','y','phi',
            'xvar','yvar','phivar','xycovar','xphicovar','yphicovar'
        ])

        # Ensure files are closed on shutdown
        rospy.on_shutdown(self.close_files)

    def odom_callback(self, msg):
        z = np.array([[msg.x], [msg.y], [msg.phi]])
        R = np.array([[msg.xvar, msg.xycovar, msg.xphicovar],
                      [msg.xycovar, msg.yvar, msg.yphicovar],
                      [msg.xphicovar, msg.yphicovar, msg.phivar]])
        # If first measurement, initialize state
        if not self.initialized:
            self.x = z
            self.P = R.copy()
            self.initialized = True
        else:
            # Predict step (identity dynamics)
            self.P = self.P + self.Q
            # Update step
            S = self.P + R
            K = np.dot(self.P, np.linalg.inv(S))
            y = z - self.x
            self.x = self.x + np.dot(K, y)
            self.P = np.dot(np.eye(3) - K, self.P)

        # Log odometry data
        t = rospy.get_time()
        self.all_writer.writerow([
            'odom', t,
            msg.x, msg.y, msg.phi,
            msg.xvar, msg.yvar, msg.phivar,
            msg.xycovar, msg.xphicovar, msg.yphicovar
        ])

        self.publish_state()

    def aruco_callback(self, msg):
        if not self.initialized:
            return
        # Save prior state for displacement check
        x_prior = self.x.copy()
        z = np.array([[msg.x], [msg.y], [msg.phi]])
        R = np.array([[msg.xvar, msg.xycovar, msg.xphicovar],
                      [msg.xycovar, msg.yvar, msg.yphicovar],
                      [msg.xphicovar, msg.yphicovar, msg.phivar]])
        # Predict step
        self.P = self.P + self.Q
        # Update step
        S = self.P + R
        K = np.dot(self.P, np.linalg.inv(S))
        y = z - self.x
        self.x = self.x + np.dot(K, y)
        # Compute Euclidean shift from prior
        dx = self.x[0,0] - x_prior[0,0]
        dy = self.x[1,0] - x_prior[1,0]
        shift = np.hypot(dx, dy)
        if shift > 50:
            rospy.loginfo("Aruco update shifted pose by %.3f m", shift)
        self.P = np.dot(np.eye(3) - K, self.P)

        # Log aruco measurement
        t = rospy.get_time()
        self.all_writer.writerow([
            'aruco', t,
            msg.x, msg.y, msg.phi,
            msg.xvar, msg.yvar, msg.phivar,
            msg.xycovar, msg.xphicovar, msg.yphicovar
        ])

        self.publish_state()

    def publish_state(self):
        msg = PoseCovar()
        msg.x = float(self.x[0, 0])
        msg.y = float(self.x[1, 0])
        msg.phi = float(self.x[2, 0])
        msg.xvar = float(self.P[0, 0])
        msg.yvar = float(self.P[1, 1])
        msg.phivar = float(self.P[2, 2])
        msg.xycovar = float(self.P[0, 1])
        msg.xphicovar = float(self.P[0, 2])
        msg.yphicovar = float(self.P[1, 2])
        self.fused_pub.publish(msg)


        # Log fused KF output
        t = rospy.get_time()
        self.all_writer.writerow([
            'fused', t,
            msg.x, msg.y, msg.phi,
            msg.xvar, msg.yvar, msg.phivar,
            msg.xycovar, msg.xphicovar, msg.yphicovar
        ])

    def close_files(self):
        self.all_csv.close()
        

def main():
    rospy.init_node('kf_fusion_node')
    node = KFNode()
    rospy.spin()

if __name__ == '__main__':
    main()