"""Publishes TF from actual Gazebo differential-drive odometry for SLAM/Nav2."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class OdomTf(Node):
    def __init__(self):
        super().__init__('ambulance_odom_tf')
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(Odometry, '/ambulance/odom', self.on_odom, 20)
        stamp = self.get_clock().now().to_msg()
        footprint = TransformStamped(); footprint.header.stamp = stamp
        footprint.header.frame_id = 'base_link'; footprint.child_frame_id = 'base_footprint'
        footprint.transform.rotation.w = 1.0
        sensor = TransformStamped(); sensor.header.stamp = stamp
        sensor.header.frame_id = 'base_link'; sensor.child_frame_id = 'lidar_link'
        sensor.transform.translation.x = 3.0; sensor.transform.translation.z = 0.20
        sensor.transform.rotation.w = 1.0
        self.static_tf.sendTransform([footprint, sensor])
    def on_odom(self, msg):
        tf = TransformStamped(); tf.header = msg.header
        tf.header.frame_id = 'odom'; tf.child_frame_id = 'base_link'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)
def main():
    rclpy.init(); node = OdomTf(); rclpy.spin(node); node.destroy_node(); rclpy.try_shutdown()
