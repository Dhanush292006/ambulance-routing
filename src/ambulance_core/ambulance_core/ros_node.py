"""ROS digital twin driven by actual Gazebo odometry and sensor observations."""
import json
import math
from pathlib import Path
import time

from .engine import DigitalTwin, Obstacle, NODES


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool
    from nav_msgs.msg import OccupancyGrid, Odometry

    class TwinNode(Node):
        def __init__(self):
            super().__init__('digital_twin_node')
            self.twin = DigitalTwin()
            self.obstacles = []
            self.pose = None
            self.last_pose = None
            self.last_time = time.monotonic()
            self.status = self.create_publisher(String, '/digital_twin/status', 10)
            self.route = self.create_publisher(String, '/digital_twin/route', 10)
            self.create_subscription(String, '/ambulance/obstacles', self.on_obstacles, 10)
            self.create_subscription(Bool, '/ambulance/emergency', self.on_emergency, 10)
            self.create_subscription(String, '/digital_twin/destination', self.on_destination, 10)
            self.create_subscription(Odometry, '/ambulance/odom', self.on_odom, 20)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
            self.create_timer(1., self.tick)

        def on_odom(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            now = time.monotonic()
            old_speed = self.twin.speed
            speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            elapsed = max(0., now - self.last_time)
            moved = math.hypot(p.x-self.last_pose[0], p.y-self.last_pose[1]) if self.last_pose else 0.
            if self.last_pose:
                self.twin.distance += moved
                self.twin.account_energy(moved, elapsed, abs(speed-old_speed) / elapsed if elapsed > 0 else 0.)
            self.last_pose = (p.x, p.y)
            self.pose = {'x': p.x, 'y': p.y, 'yaw': yaw}
            self.twin.speed = speed
            self.last_time = now
            self.twin.position = min(NODES, key=lambda n: math.hypot(NODES[n][0]-p.x, NODES[n][1]-p.y))

        def on_destination(self, msg):
            try:
                data = json.loads(msg.data)
                goal = data['goal_node']
                if goal not in NODES: raise ValueError(f'Unknown planner goal node: {goal}')
                self.twin.plan(f'destination selected: {data.get("id", goal)}', goal=goal)
                self.publish_route()
            except Exception as exc:
                self.get_logger().error(f'Invalid destination request: {exc}')

        def on_obstacles(self, msg):
            try:
                self.obstacles = [Obstacle(**x) for x in json.loads(msg.data)]
                self.twin.update_obstacles(self.obstacles)
                self.twin.plan('live LiDAR/prediction update', start=self.twin.position)
                self.publish_route()
            except Exception as exc:
                self.get_logger().warning(f'Invalid obstacle message: {exc}')

        def on_emergency(self, msg):
            self.twin.emergency = msg.data
            if self.pose:
                self.twin.plan('emergency mode changed', start=self.twin.position)
                self.publish_route()

        def on_map(self, msg):
            data = {'frame_id': msg.header.frame_id, 'resolution': msg.info.resolution,
                    'width': msg.info.width, 'height': msg.info.height,
                    'origin': [msg.info.origin.position.x, msg.info.origin.position.y],
                    'data': list(msg.data)}
            self.write_json('slam_map.json', data)

        def write_json(self, name, data):
            output = Path('results/latest')
            output.mkdir(parents=True, exist_ok=True)
            (output / name).write_text(json.dumps(data, indent=2))

        def publish_route(self):
            nodes = [edge.b for edge in self.twin.route]
            data = {'nodes': nodes, 'edges': [edge.key() for edge in self.twin.route],
                    'goal': self.twin.goal, 'replans': self.twin.replans,
                    'reason': self.twin.events[-1]['event'] if self.twin.events else ''}
            self.route.publish(String(data=json.dumps(data)))

        def tick(self):
            # Do not advance the standalone kinematic demo here. Every position,
            # speed, and distance value comes from Gazebo's odometry topic.
            self.twin.t += 1.0
            if self.pose and int(self.twin.t) % 5 == 0:
                self.twin.log(
                    f"Gazebo odometry x={self.pose['x']:.2f} m y={self.pose['y']:.2f} m "
                    f"speed={self.twin.speed:.2f} m/s; battery={self.twin.battery:.1f}%"
                )
            data = self.twin.snapshot()
            data['pose'] = self.pose
            data['position'] = (f"({self.pose['x']:.1f}, {self.pose['y']:.1f}) m" if self.pose else 'awaiting odometry')
            data['eta_s'] = (sum(edge.travel_time() for edge in self.twin.route) if self.twin.route else 0.0)
            payload = json.dumps(data)
            self.status.publish(String(data=payload))
            self.write_json('metrics.json', data)
            self.write_json('events.json', self.twin.events[-100:])
            self.write_json('route_history.json', [edge.__dict__ for edge in self.twin.edges])

    rclpy.init()
    node = TwinNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
