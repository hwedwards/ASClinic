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
        [4, 0],
        [0, 4]
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
        # odometry testing 
        # ([0, 0, 4000, 0], [0, 0, 0, 0], 18), # 0
        # ([4000, 0, 0, 0], [0, 0, 0, 0], 18), # 1
        ([0, 0, -600, -2400], [0, 0, 0, 0], 10), # 1
        ([-600, -2400, 1000, -3100], [0, 0, 0, 0], 10), #2
        ([1000, -3100, 2200, -2200], [0, 0, 300*np.cos(30), 300*np.sin(30)], 8), #2.5
        ([2200, -2200, 3600, -1600], [300*np.cos(30), 300*np.sin(30), 0, 0], 12), #3
        ([3600, -1600, 4200, -2200], [0, 0, 0, 0], 8), #4
        ([4200, -2200, 5000, -3800], [0, 0, 0, 0], 10), #5
        ([5000, -3800, 6600, -2000], [0, 0, 0, 0], 13), #6
        ([6600, -2000, 6600, -400], [0, 0, 0, 0], 10), #6.5
        ([6600, -400, 6000, 500], [0, 0, 0, 0], 10), #7
        ([6000, 500, 5000, 200], [0, 0, 0, 0], 8), # 8
        ([5000, 200, 4000, 1400], [0, 0, 0, 0], 9), # 9
        ([4000, 1400, 4200, 3400], [0, 0, 0, 0], 12), # 10
        ([4200, 3400, 5000, 4800], [0, 0, 0, 0], 10), #11
        ([5000, 4800, 600, 4600], [0, 0, 0, 0], 24), #12
        ([600, 4600, 0, 3200], [0, 0, 0, 0], 10), #13
        ([0, 3200, -1200, 500], [0, 0, 0, 0], 16), #14
        ([-1200, 500, 0, 0 ], [0, 0, 0, 0], 9) #15
    ]
    phi_list = []
    coeffx_list = []
    coeffy_list = []
    tf_list = []
    K_list = []
    for idx, (coords, vels, tf) in enumerate(segments):
        coeffx, coeffy = get_traj(coords, vels, tf)
        phi = atan2((coords[3] - coords[1]), (coords[2] - coords[0])) #phi needs to be in radians here to calculate gains
        v = 0.3 # m/s (or set per segment if needed)
        A, B = calculateLinAB(v, phi)
        A_aug, B_aug = calculateAugmented(A, B)
        K, S, E = calculateGainsK(A_aug, B_aug)
        phi_deg = phi * 180 / np.pi  # Convert to degrees for storage
        print(f"Segment {idx}: phi = {phi_deg} degrees")
        phi_list.append(phi_deg)
        coeffx_list.append(coeffx)
        coeffy_list.append(coeffy)
        tf_list.append(tf)
        K_list.append(K)
    # Save K matrix for all segments
    with open('trajectoryGainsK.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([f'segment', f'K_0_0', f'K_0_1', f'K_0_2', f'K_0_3', f'K_0_4',
                         f'K_1_0', f'K_1_1', f'K_1_2', f'K_1_3', f'K_1_4'])
        for idx, K in enumerate(K_list):
            writer.writerow([idx] + list(K.flatten()))
    # Save coefficients to CSV for ROS trajectory tracking node
    with open('trajectory_coeffs.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["FLAG", 'coeffx_0', 'coeffx_1', 'coeffx_2', 'coeffx_3', 'coeffy_0', 'coeffy_1', 'coeffy_2', 'coeffy_3', 'tf', "start_phi", "segment_no"])
        for idx, (coeffx, coeffy, tf, phi) in enumerate(zip(coeffx_list, coeffy_list, tf_list, phi_list)):
            writer.writerow([0] + list(coeffx) + list(coeffy) + [tf] + [phi] + [idx])

if __name__ == "__main__":
    main()
