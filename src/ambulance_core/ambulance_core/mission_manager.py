"""Dashboard mission manager: DT-AAGR selects waypoints, Nav2 drives each leg."""
import json
import math
from pathlib import Path
import time

from .engine import NODES

DESTINATIONS = {
    'HOSPITAL-01': {'name': 'Main Emergency Hospital', 'goal_node': 'HOSPITAL'},
    'HOSPITAL-02': {'name': 'Government Hospital', 'goal_node': 'N3'},
    'HOSPITAL-03': {'name': 'Trauma Centre', 'goal_node': 'S3'},
    'HOSPITAL-04': {'name': 'Multi-Speciality Hospital', 'goal_node': 'N2'},
    'HOSPITAL-05': {'name': 'City Medical Centre', 'goal_node': 'S1'},
    'STATION': {'name': 'Ambulance Station', 'goal_node': 'STATION'},
}


def main():
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    class MissionManager(Node):
        def __init__(self):
            super().__init__('ambulance_mission_manager')
            self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.emergency_pub = self.create_publisher(Bool, '/ambulance/emergency', 10)
            self.destination_pub = self.create_publisher(String, '/digital_twin/destination', 10)
            self.create_subscription(String, '/dashboard/mission_command', self.command, 10)
            self.create_subscription(String, '/digital_twin/route', self.on_route, 10)
            self.create_subscription(Odometry, '/ambulance/odom', self.on_odom, 20)
            self.create_subscription(String, '/digital_twin/status', self.on_twin_status, 10)
            self.destination = None
            self.goal = None
            self.waypoints = []
            self.current_waypoint = None
            self.goal_handle = None
            self.state = 'IDLE'
            self.pose = None
            self.start_time = None
            self.distance_m = 0.0
            self.previous_xy = None
            self.route_signature = None
            self.replans = 0
            self.obstacle_ids = set()
            self.remaining_distance_m = 0.0
            self.write_status()

        def write_status(self, error=None):
            current = self.destination or {}
            data = {'state': self.state, 'destination': current, 'goal': self.goal,
                    'pose': self.pose, 'speed_mps': self.speed if hasattr(self, 'speed') else 0.0,
                    'distance_m': self.distance_m, 'remaining_distance_m': self.remaining_distance_m,
                    'eta_s': self.remaining_distance_m / max(0.5, getattr(self, 'speed', 0.0)),
                    'obstacles_encountered': len(self.obstacle_ids), 'replans': self.replans,
                    'current_waypoint': self.current_waypoint, 'route_nodes': self.waypoints,
                    'elapsed_s': time.time() - self.start_time if self.start_time else 0,
                    'updated_at': time.time(), 'error': error}
            out = Path('results/latest'); out.mkdir(parents=True, exist_ok=True)
            (out / 'mission_status.json').write_text(json.dumps(data, indent=2))

        def on_odom(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.pose = {'x': p.x, 'y': p.y,
                         'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
            self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            if self.previous_xy:
                self.distance_m += math.hypot(p.x-self.previous_xy[0], p.y-self.previous_xy[1])
            self.previous_xy = (p.x, p.y)
            if self.waypoints:
                self.remaining_distance_m = math.hypot(p.x-NODES[self.waypoints[0]][0], p.y-NODES[self.waypoints[0]][1])
                self.remaining_distance_m += sum(math.hypot(NODES[a][0]-NODES[b][0], NODES[a][1]-NODES[b][1]) for a,b in zip(self.waypoints, self.waypoints[1:]))

        def on_twin_status(self, msg):
            try:
                status = json.loads(msg.data)
                self.obstacle_ids.update(o['id'] for o in status.get('obstacles', []))
                self.write_status()
            except (ValueError, KeyError):
                pass

        def command(self, msg):
            try:
                data = json.loads(msg.data)
                action = data.get('action')
                if action in ('start', 'return'):
                    if self.goal_handle: self.goal_handle.cancel_goal_async()
                    key = 'STATION' if action == 'return' else data.get('destination_id')
                    if key not in DESTINATIONS: raise ValueError('Unknown destination id')
                    self.destination = DESTINATIONS[key] | {'id': key}
                    self.goal = self.destination['goal_node']
                    self.waypoints = []
                    self.current_waypoint = None
                    self.goal_handle = None
                    self.route_signature = None
                    self.start_time = time.time()
                    self.distance_m = 0.0
                    self.obstacle_ids.clear()
                    self.state = 'PLANNING'
                    self.emergency_pub.publish(Bool(data=key != 'STATION'))
                    request = {'id': key, 'goal_node': self.goal}
                    self.destination_pub.publish(String(data=json.dumps(request)))
                    self.write_status()
                elif action == 'pause':
                    self.state = 'PAUSED'
                    if self.goal_handle: self.goal_handle.cancel_goal_async()
                    self.write_status()
                elif action == 'resume':
                    if not self.destination: raise ValueError('No mission to resume')
                    self.state = 'PLANNING'; self.goal_handle = None
                    self.current_waypoint = None; self.route_signature = None
                    self.destination_pub.publish(String(data=json.dumps({'id': self.destination['id'], 'goal_node': self.goal})))
                    self.write_status()
                elif action == 'cancel':
                    self.state = 'CANCELLED'
                    if self.goal_handle: self.goal_handle.cancel_goal_async()
                    self.emergency_pub.publish(Bool(data=False)); self.write_status()
                else:
                    raise ValueError('Unsupported mission action')
            except Exception as exc:
                self.state = 'NAVIGATION FAILED'; self.write_status(str(exc))
                self.get_logger().error(str(exc))

        def on_route(self, msg):
            if self.state not in ('PLANNING', 'NAVIGATING', 'REPLANNING'): return
            try:
                route = json.loads(msg.data)
                if route.get('goal') != self.goal: return
                nodes = route.get('nodes', [])
                signature = tuple(nodes)
                if not nodes:
                    self.state = 'NAVIGATION FAILED'; self.write_status('DT-AAGR has no safe route'); return
                if signature == self.route_signature: return
                self.route_signature = signature
                self.waypoints = list(nodes)
                if self.current_waypoint in self.waypoints:
                    self.waypoints = self.waypoints[self.waypoints.index(self.current_waypoint):]
                if self.pose:
                    self.remaining_distance_m = math.hypot(self.pose['x']-NODES[nodes[0]][0], self.pose['y']-NODES[nodes[0]][1])
                    self.remaining_distance_m += sum(math.hypot(NODES[a][0]-NODES[b][0], NODES[a][1]-NODES[b][1]) for a,b in zip(nodes,nodes[1:]))
                self.replans = max(self.replans, int(route.get('replans', 0)))
                if self.current_waypoint and self.current_waypoint not in self.waypoints:
                    self.state = 'REPLANNING'
                    if self.goal_handle: self.goal_handle.cancel_goal_async()
                elif not self.goal_handle:
                    self.dispatch_next()
                self.write_status()
            except Exception as exc:
                self.state = 'NAVIGATION FAILED'; self.write_status(str(exc))

        def dispatch_next(self):
            if self.state in ('PAUSED', 'CANCELLED', 'ARRIVED', 'NAVIGATION FAILED'): return
            if not self.waypoints:
                self.state = 'ARRIVED'
                self.emergency_pub.publish(Bool(data=False))
                self.write_status()
                return
            if not self.client.wait_for_server(timeout_sec=0.1):
                self.state = 'NAVIGATION FAILED'
                self.write_status('Nav2 navigate_to_pose action server is unavailable')
                return
            node_name = self.waypoints[0]
            x, y = NODES[node_name]
            pose = PoseStamped(); pose.header.frame_id = 'map'
            pose.pose.position.x = float(x); pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            request = NavigateToPose.Goal(); request.pose = pose
            self.current_waypoint = node_name
            self.state = 'NAVIGATING'
            future = self.client.send_goal_async(request, feedback_callback=self.feedback)
            future.add_done_callback(self.goal_response)
            self.write_status()

        def goal_response(self, future):
            handle = future.result()
            if not handle or not handle.accepted:
                self.goal_handle = None
                self.state = 'NAVIGATION FAILED'; self.write_status('Nav2 rejected a DT-AAGR waypoint'); return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.goal_result)

        def feedback(self, _feedback):
            self.write_status()

        def goal_result(self, future):
            status = future.result().status
            self.goal_handle = None
            if status == GoalStatus.STATUS_CANCELED:
                if self.state == 'PAUSED' or self.state == 'CANCELLED': return
                if self.state == 'REPLANNING':
                    self.current_waypoint = None
                    self.dispatch_next()
                return
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.state = 'NAVIGATION FAILED'; self.write_status('Nav2 waypoint failed'); return
            if self.current_waypoint in self.waypoints:
                self.waypoints = self.waypoints[self.waypoints.index(self.current_waypoint)+1:]
            if not self.waypoints:
                self.state = 'ARRIVED'
                self.emergency_pub.publish(Bool(data=False))
                self.write_status()
                self.record_result()
            else:
                self.current_waypoint = None
                self.dispatch_next()

        def record_result(self):
            data = {'destination': self.destination, 'travel_time_s': time.time()-self.start_time,
                    'path_length_m': self.distance_m, 'replans': self.replans,
                    'estimated_energy_used_pct': min(100.0, self.distance_m*.004),
                    'obstacles_encountered': len(self.obstacle_ids), 'route_changes': max(0,self.replans-1),
                    'navigation_status': self.state}
            out = Path('results/latest'); out.mkdir(parents=True, exist_ok=True)
            (out/'mission_result.json').write_text(json.dumps(data, indent=2))

    rclpy.init(); node = MissionManager()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__': main()
