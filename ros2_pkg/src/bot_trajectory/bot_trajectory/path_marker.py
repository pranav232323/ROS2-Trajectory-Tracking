#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PathMarker(Node):
    def __init__(self):
        super().__init__('path_marker')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Marker, '/path_marker', 10)

        # Create marker object
        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.ns = "path"
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05  # line thickness
        self.marker.color.a = 1.0   # alpha
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0   # blue ink

        self.get_logger().info("TurtleBot ink trail started!")

    def odom_callback(self, msg):
        p = Point()
        p.x = msg.pose.pose.position.x
        p.y = msg.pose.pose.position.y
        p.z = 0.01  # slightly above ground
        self.marker.points.append(p)

        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = PathMarker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
