#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class LidarObstacleDetector(Node):

    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        self.declare_parameter('detection_distance', 4.0)
        self.declare_parameter('front_angle_deg', 70.0)
        self.declare_parameter('min_valid_range', 0.15)

        self.distance_limit = float(
            self.get_parameter('detection_distance').value
        )

        self.front_angle = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )

        self.min_range = float(
            self.get_parameter('min_valid_range').value
        )

        self.obstacle_pub = self.create_publisher(
            Bool,
            '/ambulance/roadblock_detected',
            10
        )

        self.distance_pub = self.create_publisher(
            Float32,
            '/ambulance/obstacle_distance',
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.previous_state = False

        self.get_logger().info(
            'LiDAR obstacle detector ACTIVE'
        )

    def scan_callback(self, msg):

        minimum = float('inf')
        angle = msg.angle_min

        for r in msg.ranges:

            if math.isfinite(r) and r >= self.min_range:

                if abs(angle) <= self.front_angle:

                    if r < minimum:
                        minimum = r

            angle += msg.angle_increment

        detected = minimum <= self.distance_limit

        distance = Float32()

        if math.isfinite(minimum):
            distance.data = float(minimum)
        else:
            distance.data = -1.0

        self.distance_pub.publish(distance)

        state = Bool()
        state.data = detected
        self.obstacle_pub.publish(state)

        if detected != self.previous_state:

            if detected:
                self.get_logger().warn(
                    f'ROADBLOCK DETECTED: {minimum:.2f} m'
                )
            else:
                self.get_logger().info(
                    'ROADBLOCK CLEARED'
                )

            self.previous_state = detected


def main(args=None):

    rclpy.init(args=args)

    node = LidarObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
