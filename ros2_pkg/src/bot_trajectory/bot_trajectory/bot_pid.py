#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import time
import pandas as pd

class TrajectoryPID(Node):
    def __init__(self):
        super().__init__('trajectory_pid')
        file_path = '/home/pranav/wks_ros2/src/bot_trajectory/waypoints_csv/spline_xy_time.csv'
        df = pd.read_csv(file_path)

        # Extract x and y columns
        self.x_values = df['x'].to_numpy()
        self.y_values = df['y'].to_numpy()

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.index = 0

        # Pose
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # PID gains (tune these)
        self.Kp_lin = 0.5
        self.Ki_lin = 0.01
        self.Kd_lin = 0.1

        self.Kp_ang = 1.0
        self.Ki_ang = 0.0
        self.Kd_ang = 0.3

        # PID memory
        self.prev_dist_error = 0.0
        self.int_dist_error = 0.0

        self.prev_ang_error = 0.0
        self.int_ang_error = 0.0

        self.prev_time = time.time()

        self.timer = self.create_timer(0.05, self.track_path)
        self.get_logger().info('PID trajectory node started')

    def odom_callback(self, msg):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_pose['yaw'] = math.atan2(2.0 * (qw * qz + qx * qy),
                                              1.0 - 2.0 * (qy**2 + qz**2))

    def track_path(self):
        if self.index >= len(self.x_values):
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Trajectory complete.')
            return

        # Target point
        x_ref = self.x_values[self.index]
        y_ref = self.y_values[self.index]

        # Current position
        x = self.current_pose['x']
        y = self.current_pose['y']
        yaw = self.current_pose['yaw']

        # Compute position error
        dx = x_ref - x
        dy = y_ref - y
        distance_error = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        angular_error = target_yaw - yaw
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # Time step
        now = time.time()
        dt = now - self.prev_time
        if dt == 0: dt = 1e-3

        # --- Linear PID ---
        self.int_dist_error += distance_error * dt
        der_dist_error = (distance_error - self.prev_dist_error) / dt
        
        v = (self.Kp_lin * distance_error +
             self.Ki_lin * self.int_dist_error +
             self.Kd_lin * der_dist_error)

        # --- Angular PID ---
        self.int_ang_error += angular_error * dt
        der_ang_error = (angular_error - self.prev_ang_error) / dt
        w = (self.Kp_ang * angular_error +
             self.Ki_ang * self.int_ang_error +
             self.Kd_ang * der_ang_error)

        # Limit speeds
        v = max(min(v, 0.3), -0.3)
        w = max(min(w, 0.5), -0.5)

        # Publish
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f'Target ({x_ref:.2f},{y_ref:.2f}) | Pose ({x:.2f},{y:.2f}) | v={v:.2f}, w={w:.2f}')

        # Save previous for next iteration
        self.prev_time = now
        self.prev_dist_error = distance_error
        self.prev_ang_error = angular_error

        # Go to next point when close enough
        if distance_error < 0.1:
            self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPID()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
