#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')

import re
import sys
import matplotlib.pyplot as plt

def parse_log(filename):
    times_w = []
    vals_w = []
    times_err = []
    vals_err = []
    times_phi = []
    vals_phi = []

    # Patterns to extract timestamps and values
    re_err = re.compile(r'\[\s*INFO\]\s*\[(\d+\.\d+)\]:\s*error_y:\s*([-\d\.]+)')
    re_w   = re.compile(r'\[\s*INFO\]\s*\[(\d+\.\d+)\]:\s*w:\s*([-\d\.]+)')
    # Pattern to extract phi (heading)
    re_phi = re.compile(r'\[\s*INFO\]\s*\[(\d+\.\d+)\].*?phi:\s*([-\d\.]+)')

    with open(filename, 'r') as f:
        for line in f:
            m_err = re_err.search(line)
            if m_err:
                times_err.append(float(m_err.group(1)))
                vals_err.append(float(m_err.group(2)))
            m_w = re_w.search(line)
            if m_w:
                times_w.append(float(m_w.group(1)))
                vals_w.append(float(m_w.group(2)))
            m_phi = re_phi.search(line)
            if m_phi:
                times_phi.append(float(m_phi.group(1)))
                vals_phi.append(float(m_phi.group(2)))

    return times_w, vals_w, times_err, vals_err, times_phi, vals_phi

def main():
    if len(sys.argv) != 2:
        print("Usage: python plot_output_log.py <log_filename>")
        sys.exit(1)

    log_filename = sys.argv[1]
    t_w, w_vals, t_err, err_vals, t_phi, phi_vals = parse_log(log_filename)
    
    if not t_w and not t_err and not t_phi:
        print("No data found in log.")
        sys.exit(1)

    # Align time to start at zero
    all_times = t_w + t_err + t_phi
    t0 = min(all_times)
    t_w = [t - t0 for t in t_w]
    t_err = [t - t0 for t in t_err]
    t_phi = [t - t0 for t in t_phi]

    # Plot w (angular velocity) over time
    plt.figure()
    plt.plot(t_w, w_vals, linestyle='-')
    plt.xlabel('Time [s]')
    plt.ylabel('w (angular velocity)')
    plt.title('Angular Velocity w over Time')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('z_figures/w_plot.png')
    print("Saved plot to w_plot.png")

    # Plot error_y (lateral error) over time
    plt.figure()
    plt.plot(t_err, err_vals, linestyle='-')
    plt.xlabel('Time [s]')
    plt.ylabel('error_y (lateral error)')
    plt.title('Lateral Error error_y over Time')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('z_figures/error_y_plot.png')
    print("Saved plot to error_y_plot.png")

    # Plot phi (heading) over time
    plt.figure()
    plt.plot(t_phi, phi_vals, linestyle='-')
    plt.xlabel('Time [s]')
    plt.ylabel('phi (heading)')
    plt.title('Heading phi over Time')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('z_figures/phi_plot.png')
    print("Saved plot to phi_plot.png")

if __name__ == "__main__":
    main()