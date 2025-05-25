import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import csv
import control
from math import acos, sin, cos, sqrt, atan2

# Define coordinates and velocities [x0; y0; xf; yf] and [vx0; vy0; vxf; vyf]
def calculateLinAB(v, phi):
    A = np.array([
        [0, 0, -v*np.sin(phi)],
        [0, 0,  v*np.cos(phi)],
        [0, 0, 0]
    ])
    B = np.array([
        [np.cos(phi), 0],
        [np.sin(phi), 0],
        [0, 1]
    ])
    return A, B
def calculateAugmented(A, B):
    A_aug = np.block([
        [A, np.zeros((3,2))],
        [np.eye(2), np.zeros((2,3))]
    ])
    B_aug = np.block([
        [B],
        [np.zeros((2,2))]
    ])
    return A_aug, B_aug
def calculateGainsK(A_aug, B_aug):
    Q = np.array([
        [10, 0, 0, 0, 0],
        [0, 10, 0, 0, 0],
        [0, 0, 10, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])
    R = np.array([
        [40, 0],
        [0, 40]
    ])
    K = control.lqr(A_aug, B_aug, Q, R)
    return K
def get_traj(coords, vels, tf):
    """
    Computes cubic trajectory coefficients for x and y.
    coords: [x0, y0, xf, yf] (mm)
    vels: [vx0, vy0, vxf, vyf] (mm/s)
    tf: final time (s)
    Returns: coeffx, coeffy (each is [a0, a1, a2, a3] for a0 + a1*t + a2*t^2 + a3*t^3)
    """
    A = np.array([
        [1, 0,    0,      0],
        [0, 1,    0,      0],
        [1, tf, tf**2, tf**3],
        [0, 1,  2*tf,  3*tf**2]
    ])
    Bx = np.array([coords[0], vels[0], coords[2], vels[2]])
    By = np.array([coords[1], vels[1], coords[3], vels[3]])
    coeffx = np.linalg.solve(A, Bx)
    coeffy = np.linalg.solve(A, By)
    return coeffx, coeffy

    
def main():
    # Example: define multiple line segments
    segments = [
        # Each tuple: (coords, vels, tf)
        ([0, 0, 5000, 0], [0, 0, 0, 0], 20),
        ([5000, 0, 5000, 2000], [0, 0, 0, 0], 10),
        ([5000, 2000, 0, 2000], [0, 0, 0, 0], 20),
        ([0, 2000, 0, 0], [0, 0, 0, 0], 10)
    ]
    phi_list = []
    coeffx_list = []
    coeffy_list = []
    tf_list = []
    K_list = []
    for idx, (coords, vels, tf) in enumerate(segments):
        coeffx, coeffy = get_traj(coords, vels, tf)
        phi = atan2((coords[3] - coords[1]), (coords[2] - coords[0]))
        v = 0.3 # m/s (or set per segment if needed)
        A, B = calculateLinAB(v, phi)
        A_aug, B_aug = calculateAugmented(A, B)
        K, S, E = calculateGainsK(A_aug, B_aug)
        phi_list.append(phi)
        coeffx_list.append(coeffx)
        coeffy_list.append(coeffy)
        tf_list.append(tf)
        K_list.append(K)
    # Save K matrix for the first segment (or average, or all if needed)
    with open('trajectoryGainsK.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([f'K_{i}_{j}' for i in range(K_list[0].shape[0]) for j in range(K_list[0].shape[1])])
        writer.writerow(K_list[0].flatten())
    # Save coefficients to CSV for ROS trajectory tracking node
    with open('trajectory_coeffs.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["FLAG", 'coeffx_0', 'coeffx_1', 'coeffx_2', 'coeffx_3', 'coeffy_0', 'coeffy_1', 'coeffy_2', 'coeffy_3', 'tf', "start_phi", "segment_no"])
        for idx, (coeffx, coeffy, tf, phi) in enumerate(zip(coeffx_list, coeffy_list, tf_list, phi_list)):
            writer.writerow([0] + list(coeffx) + list(coeffy) + [tf] + [phi] + [idx])

if __name__ == "__main__":
    main()
