#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')

import re
import sys
import matplotlib.pyplot as plt

def parse_log(filename):
    times = []
    ref_x = []
    ref_y = []
    state_x = []
    state_y = []

    # Updated regex to extract timestamp and values
    ref_re = re.compile(r'\[(\d+\.\d+)\] Reference trajectory - x: ([\d\.-]+), y: ([\d\.-]+), phi: ([\d\.-]+), v: ([\d\.-]+), w: ([\d\.-]+)')
    state_re = re.compile(r'\[(\d+\.\d+)\] Robot state - x: ([\d\.-]+), y: ([\d\.-]+), phi: ([\d\.-]+)')

    ref_times = []
    state_times = []

    with open(filename, 'r') as f:
        lines = f.readlines()

    for line in lines:
        ref_match = ref_re.search(line)
        state_match = state_re.search(line)
        if ref_match:
            ref_times.append(float(ref_match.group(1)))
            ref_x.append(float(ref_match.group(2)))
            ref_y.append(float(ref_match.group(3)))
        if state_match:
            state_times.append(float(state_match.group(1)))
            state_x.append(float(state_match.group(2)))
            state_y.append(float(state_match.group(3)))

    return state_times, ref_times, ref_x, ref_y, state_x, state_y

def main():
    if len(sys.argv) != 2:
        print("Usage: python plot_log.py <log_filename>")
        sys.exit(1)

    log_filename = sys.argv[1]
    state_times, ref_times, ref_x, ref_y, state_x, state_y = parse_log(log_filename)
    
    if not state_times:
        print("No robot state data found in log.")
        sys.exit(1)

    # Pad or truncate reference arrays to match robot state time length
    min_len = min(len(state_x), len(state_y), len(state_times))
    state_x = state_x[:min_len]
    state_y = state_y[:min_len]
    state_times = state_times[:min_len]

    # Interpolate reference x/y to robot state timestamps
    import numpy as np
    if len(ref_times) > 1:
        ref_x_interp = np.interp(state_times, ref_times, ref_x)
        ref_y_interp = np.interp(state_times, ref_times, ref_y)
    else:
        ref_x_interp = [ref_x[0]] * min_len
        ref_y_interp = [ref_y[0]] * min_len

    # Align time to start at zero
    t0 = min(state_times)
    state_times = [t - t0 for t in state_times]

    # Plot x positions over time
    plt.figure()
    plt.plot(state_times, ref_x_interp, label='Reference x')
    plt.plot(state_times, state_x, label='Robot x')
    plt.xlabel('Time [s]')
    plt.ylabel('x position [m]')
    plt.title('X Position over Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('z_figures/x_position_plot.png')
    print("Saved plot to x_position_plot.png")

    # Plot y positions over time
    plt.figure()
    plt.plot(state_times, ref_y_interp, label='Reference y')
    plt.plot(state_times, state_y, label='Robot y')
    plt.xlabel('Time [s]')
    plt.ylabel('y position [m]')
    plt.title('Y Position over Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('z_figures/y_position_plot.png')
    print("Saved plot to y_position_plot.png")

if __name__ == "__main__":
    main()