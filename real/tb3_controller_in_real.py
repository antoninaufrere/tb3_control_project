#!/usr/bin/env python3
import sys
import math
import csv
import os
import random
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

class Tb3Controller(Node):
    def __init__(self, traj_type: str):
        super().__init__('tb3_controller_real')
        self.traj_type = traj_type
        self.get_logger().info(f"Trajectory : {self.traj_type}")

        # Kinematic Gains
        self.K1 = 1.0
        self.K2 = 4.0
        self.K3 = 2.0

        # Speed limits
        self.v_max = 0.22
        self.w_max = 2.84

        # Odom
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.odom_ready = False

        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.log = []

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)

    # Odom
    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.pose = Pose2D(x, y, yaw)
        if not self.odom_ready:
            self.get_logger().info("Premier message /odom reçu.")
        self.odom_ready = True

    # Control loop
    def control_loop(self):
        if not self.odom_ready:
            return

        t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time

        xr, yr, dxr, dyr, xddr, yddr = self.ref_xy(t)
        phi_r = math.atan2(dyr, dxr)
        v_r = math.hypot(dxr, dyr)
        den = max(v_r ** 2, 1e-8)
        w_r = (dxr * yddr - dyr * xddr) / den

        x, y, phi = self.pose.x, self.pose.y, self.pose.yaw
        e1, e2, e3 = self.posture_error(x, y, phi, xr, yr, phi_r)

        # Without disturbance
        eta_v = v_r * math.cos(e3) + self.K1 * e1
        eta_w = w_r + v_r * (self.K2 * e2 + self.K3 * math.sin(e3))

        # Disturbances
        
        v_cmd = max(min(eta_v, self.v_max), -self.v_max)
        w_cmd = max(min(eta_w, self.w_max), -self.w_max)

        msg = Twist()
        msg.linear.x = v_cmd
        msg.angular.z = w_cmd
        self.cmd_pub.publish(msg)

        self.log.append([t, x, y, phi, xr, yr])

    def stop_robot(self):
        try:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info("Stopping robot.")
        except Exception as e:
            self.get_logger().warn(
                f"Error: {e}"
            )

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def posture_error(x, y, phi, xr, yr, phi_r):
        dx = xr - x
        dy = yr - y
        dphi = Tb3Controller.wrap_angle(phi_r - phi)

        c = math.cos(phi)
        s = math.sin(phi)

        e1 = c * dx + s * dy
        e2 = -s * dx + c * dy
        e3 = dphi
        return e1, e2, e3

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def ref_xy(self, t: float):
        if self.traj_type == 'line':
            vx = 0.05
            vy = 0.0
            xr = vx * t - 0.15
            yr = 0.0
            dxr = vx
            dyr = vy
            xddr = 0.0
            yddr = 0.0

        elif self.traj_type == 'circle':
            R0 = 0.30
            omega = 0.40
            xr = R0 * math.cos(omega * t)
            yr = R0 * math.sin(omega * t)
            dxr = -R0 * omega * math.sin(omega * t)
            dyr =  R0 * omega * math.cos(omega * t)
            xddr = -R0 * omega ** 2 * math.cos(omega * t)
            yddr = -R0 * omega ** 2 * math.sin(omega * t)

        elif self.traj_type == 'eight':
            Ax = 0.30
            Ay = 0.15
            w0 = 0.35
            xr = Ax * math.sin(w0 * t)
            yr = Ay * math.sin(2.0 * w0 * t)
            dxr = Ax * w0 * math.cos(w0 * t)
            dyr = 2.0 * Ay * w0 * math.cos(2.0 * w0 * t)
            xddr = -Ax * w0 ** 2 * math.sin(w0 * t)
            yddr = -4.0 * Ay * w0 ** 2 * math.sin(2.0 * w0 * t)
        else:
            xr = yr = dxr = dyr = xddr = yddr = 0.0

        return xr, yr, dxr, dyr, xddr, yddr

    # Save
    def save_log(self, filename: str):
        if not self.log:
            self.get_logger().warn("Nothing to save.")
            return
        path = os.path.expanduser(filename)
        self.get_logger().info(f"Saved into: {path}")
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['t', 'x', 'y', 'yaw', 'xr', 'yr'])
                writer.writerows(self.log)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)

    traj = None

    if len(sys.argv) >= 2:
        choice = sys.argv[1].strip().lower()
        if choice in ['1', 'line']:
            traj = 'line'
        elif choice in ['2', 'circle', 'cercle']:
            traj = 'circle'
        elif choice in ['3', 'eight', '8']:
            traj = 'eight'
        else:
            print(f" '{choice}' not known, using circle.")
            traj = 'circle'
    else:
        # Mode interactif
        print("Choose trajectory :")
        print("  1) line")
        print("  2) circle")
        print("  3) eight")
        choice = input("Your choice : ").strip().lower()

        if choice in ['1', 'line']:
            traj = 'line'
        elif choice in ['2', 'circle', 'cercle']:
            traj = 'circle'
        elif choice in ['3', 'eight', '8']:
            traj = 'eight'
        else:
            print(f"{choice}' not known, using circle.")
            traj = 'circle'

    node = Tb3Controller(traj)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\nStopping.")
    finally:
        node.stop_robot()
        node.save_log('~/tb3_traj_log.csv')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
