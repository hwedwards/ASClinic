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

    # Regular expressions to extract reference and robot state
    ref_re = re.compile(r'Reference trajectory - x: ([\d\.-]+), y: ([\d\.-]+), phi: ([\d\.-]+), v: ([\d\.-]+), w: ([\d\.-]+)')
    state_re = re.compile(r'Robot state - x: ([\d\.-]+), y: ([\d\.-]+), phi: ([\d\.-]+)')

    with open(filename, 'r') as f:
        lines = f.readlines()

    # We'll use a simple counter for time (assuming log is in order and 10 Hz)
    timestep = 0
    for line in lines:
        ref_match = ref_re.search(line)
        state_match = state_re.search(line)
        if ref_match:
            ref_x.append(float(ref_match.group(1)))
            ref_y.append(float(ref_match.group(2)))
        if state_match:
            state_x.append(float(state_match.group(1)))
            state_y.append(float(state_match.group(2)))
            times.append(timestep * 0.1)
            timestep += 1

    return times, ref_x, ref_y, state_x, state_y

def main():
    if len(sys.argv) != 2:
        print("Usage: python plot_output_log.py <log_filename>")
        sys.exit(1)

    log_filename = sys.argv[1]
    times, ref_x, ref_y, state_x, state_y = parse_log(log_filename)
    
    if not times:
        print("No data found in log.")
        sys.exit(1)

    # Truncate or pad reference arrays to match robot state time length
    min_len = min(len(state_x), len(state_y), len(times))
    state_x = state_x[:min_len]
    state_y = state_y[:min_len]
    times = times[:min_len]
    if len(ref_x) < min_len:
        ref_x = ref_x + [ref_x[-1]] * (min_len - len(ref_x))
    else:
        ref_x = ref_x[:min_len]
    if len(ref_y) < min_len:
        ref_y = ref_y + [ref_y[-1]] * (min_len - len(ref_y))
    else:
        ref_y = ref_y[:min_len]

    # Align time to start at zero
    t0 = min(times)
    times = [t - t0 for t in times]

    # Plot x positions over time
    plt.figure()
    plt.plot(times, ref_x, label='Reference x')
    plt.plot(times, state_x, label='Robot x')
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
    plt.plot(times, ref_y, label='Reference y')
    plt.plot(times, state_y, label='Robot y')
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