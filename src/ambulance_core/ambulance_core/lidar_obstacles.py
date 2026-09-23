"""Convert actual Gazebo LiDAR returns plus odometry into twin obstacle updates."""
import json, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector'); self.pose = None; self.last = None
        self.pub = self.create_publisher(String, '/ambulance/obstacles', 10)
        self.create_subscription(Odometry, '/ambulance/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
    def on_odom(self, msg):
        p=msg.pose.pose; q=p.orientation
        self.pose=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def on_scan(self, scan):
        if self.pose is None: return
        samples=[(r,scan.angle_min+i*scan.angle_increment) for i,r in enumerate(scan.ranges)
          if math.isfinite(r) and scan.range_min < r < min(scan.range_max,9.0) and abs(scan.angle_min+i*scan.angle_increment)<math.radians(35)]
        if not samples:
            if self.last is not None: self.pub.publish(String(data='[]')); self.last=None
            return
        distance,angle=min(samples); x,y,yaw=self.pose; ox=x+distance*math.cos(yaw+angle); oy=y+distance*math.sin(yaw+angle)
        state=(round(ox,1),round(oy,1))
        if state==self.last: return
        self.last=state
        self.pub.publish(String(data=json.dumps([{'id':'lidar_obstacle_01','kind':'detected road obstacle','x':ox,'y':oy,'vx':0.0,'vy':0.0,'risk':1.0}])))
        self.get_logger().info(f'LiDAR obstacle at ({ox:.1f}, {oy:.1f}), {distance:.1f} m')
def main():
    rclpy.init(); node=LidarObstacleDetector(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
