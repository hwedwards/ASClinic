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
        [0, 0, 0, 10, 0],
        [0, 0, 0, 0, 10]
    ])
    R = np.array([
        [1, 0],
        [0, 1]
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
    # Example usage for a linear trajectory in mm and seconds
    # I have found from testing that an operating speed of 0.3 m/s is pretty good. 
    coords = [0, 0, 5000, 5000]  # mm
    vels = [0, 0, 0, 0]       # mm/s
    tf = 30                    # seconds
    coeffx, coeffy = get_traj(coords, vels, tf)
    t_var = np.linspace(0, tf, 1000)
    v = 0.3 # m/s
    phi = atan2((coords[3] - coords[1]),(coords[2] - coords[0]))
    A, B = calculateLinAB(v, phi)
    A_aug, B_aug = calculateAugmented(A, B)
    K, S, E = calculateGainsK(A_aug, B_aug)
    print("K matrix: ", K)
    print("Phi", phi)

    # Save K matrix to CSV
    with open('trajectoryGainsK.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([f'K_{i}_{j}' for i in range(K.shape[0]) for j in range(K.shape[1])])
        writer.writerow(K.flatten())

    # plt.figure(figsize=(15, 4))
    # plt.subplot(1, 3, 1)
    # plt.plot(t_var, x_traj, label='x (mm)')
    # plt.xlabel('Time (s)')
    # plt.ylabel('X (mm)')
    # plt.title('Cubic Trajectory for X')
    # plt.grid(True)
    # plt.legend()

    # plt.subplot(1, 3, 2)
    # plt.plot(t_var, y_traj, label='y (mm)', color='orange')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Y (mm)')
    # plt.title('Cubic Trajectory for Y')
    # plt.grid(True)
    # plt.legend()

    # plt.subplot(1, 3, 3)
    # plt.plot(x_traj, y_traj, label='Trajectory (x vs y)', color='green')
    # plt.xlabel('X (mm)')
    # plt.ylabel('Y (mm)')
    # plt.title('Trajectory in XY Plane')
    # plt.grid(True)
    # plt.legend()

    # plt.tight_layout()
    # plt.savefig('cubic_trajectories.png')
    # plt.show()  # Optionally keep this for interactive viewing

    # Save coefficients to CSV for ROS trajectory tracking node
    with open('trajectory_coeffs.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['coeffx_0', 'coeffx_1', 'coeffx_2', 'coeffx_3', 'coeffy_0', 'coeffy_1', 'coeffy_2', 'coeffy_3', 'tf'])
        writer.writerow(list(coeffx) + list(coeffy) + [tf])

if __name__ == "__main__":
    main()
