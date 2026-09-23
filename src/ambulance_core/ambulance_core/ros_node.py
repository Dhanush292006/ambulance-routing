"""ROS bridge: the same planner used by the standalone demonstrator."""
import json
from pathlib import Path
from .engine import DigitalTwin, Obstacle
def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool
    from nav_msgs.msg import OccupancyGrid
    class TwinNode(Node):
        def __init__(self):
            super().__init__('digital_twin_node'); self.twin=DigitalTwin(); self.obstacles=[]
            self.status=self.create_publisher(String,'/digital_twin/status',10); self.route=self.create_publisher(String,'/digital_twin/route',10)
            self.map_payload = None
            self.create_subscription(String,'/ambulance/obstacles',self.on_obstacles,10); self.create_subscription(Bool,'/ambulance/emergency',self.on_emergency,10); self.create_subscription(OccupancyGrid,'/map',self.on_map,1); self.create_timer(1.,self.tick)
        def on_obstacles(self,msg):
            try: self.obstacles=[Obstacle(**x) for x in json.loads(msg.data)]; self.twin.update_obstacles(self.obstacles); self.twin.plan('ROS obstacle update')
            except Exception as e: self.get_logger().warning(f'Invalid obstacle message: {e}')
        def on_emergency(self,msg): self.twin.emergency=msg.data; self.twin.plan('mode changed')
        def on_map(self,msg):
            # Persist the measured SLAM occupancy grid for the web process.
            self.map_payload = {'frame_id':msg.header.frame_id, 'resolution':msg.info.resolution,
              'width':msg.info.width, 'height':msg.info.height,
              'origin':[msg.info.origin.position.x,msg.info.origin.position.y], 'data':list(msg.data)}
            output = Path('results/latest'); output.mkdir(parents=True, exist_ok=True)
            (output/'slam_map.json').write_text(json.dumps(self.map_payload))
        def tick(self):
            self.twin.tick(); data=json.dumps(self.twin.snapshot()); self.status.publish(String(data=data)); self.route.publish(String(data=json.dumps(self.twin.snapshot()['route']))); self.twin.write_results()
    rclpy.init(); n=TwinNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
