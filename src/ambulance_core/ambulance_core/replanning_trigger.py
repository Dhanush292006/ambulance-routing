#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class ReplanningTrigger(Node):

    def __init__(self):

        super().__init__('dt_aagr_replanning_trigger')

        self.distance = -1.0
        self.previous = False

        self.replan_pub = self.create_publisher(
            Bool,
            '/ambulance/replan_required',
            10
        )

        self.create_subscription(
            Bool,
            '/ambulance/roadblock_detected',
            self.obstacle_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/ambulance/obstacle_distance',
            self.distance_callback,
            10
        )

        self.get_logger().info(
            'DT-AAGR REPLANNING TRIGGER ACTIVE'
        )

    def distance_callback(self, msg):
        self.distance = msg.data

    def obstacle_callback(self, msg):

        detected = msg.data

        output = Bool()
        output.data = detected

        self.replan_pub.publish(output)

        if detected and not self.previous:

            self.get_logger().warn(
                '======================================'
            )

            self.get_logger().warn(
                'DT-AAGR REPLANNING REQUIRED'
            )

            self.get_logger().warn(
                f'Obstacle distance: {self.distance:.2f} m'
            )

            self.get_logger().warn(
                '======================================'
            )

        elif not detected and self.previous:

            self.get_logger().info(
                'Roadblock cleared'
            )

        self.previous = detected


def main(args=None):

    rclpy.init(args=args)

    node = ReplanningTrigger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
