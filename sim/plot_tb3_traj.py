#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import os

LOG_PATH = os.path.expanduser('~/tb3_traj_log.csv')

def main():
    if not os.path.exists(LOG_PATH):
        print(f"Log file not found : {LOG_PATH}")
        return

    ts = []
    xs = []
    ys = []
    xrs = []
    yrs = []

    with open(LOG_PATH, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            ts.append(float(row['t']))
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            xrs.append(float(row['xr']))
            yrs.append(float(row['yr']))

    plt.figure()
    plt.title("Robot Trajectory")
    plt.plot(xrs, yrs, '--', label='Reference trajectory')
    plt.plot(xs, ys, label='Odom')
    plt.axis('equal')
    plt.grid(True)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
