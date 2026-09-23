"""Send an explicit Nav2 NavigateToPose goal in the live SLAM map frame."""
import argparse, json, math, time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter

class GoalClient(Node):
    def __init__(self):
        super().__init__('ambulance_nav_goal', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.client=ActionClient(self,NavigateToPose,'navigate_to_pose'); self.map_received=False
        self.create_subscription(OccupancyGrid,'/map',lambda _: setattr(self,'map_received',True),1)
    def send(self,x,y,yaw):
        deadline=time.monotonic()+20
        while not self.map_received and time.monotonic()<deadline: rclpy.spin_once(self,timeout_sec=.2)
        if not self.map_received: raise RuntimeError('No live /map received. Start ./gazebo.sh, wait for SLAM Toolbox, then teleoperate to scan the corridor.')
        if not self.client.wait_for_server(timeout_sec=20): raise RuntimeError('Nav2 action server is not active. Start ./gazebo.sh and wait for Nav2.')
        goal=NavigateToPose.Goal(); goal.pose.header.frame_id='map'
        # A zero stamp requests Nav2's latest transform and avoids wall-clock versus /clock mismatch.
        goal.pose.pose.position.x=x; goal.pose.pose.position.y=y; goal.pose.pose.orientation.z=math.sin(yaw/2); goal.pose.pose.orientation.w=math.cos(yaw/2)
        future=self.client.send_goal_async(goal); rclpy.spin_until_future_complete(self,future)
        handle=future.result()
        if not handle or not handle.accepted: raise RuntimeError('Nav2 rejected the goal; first create enough SLAM map around the ambulance.')
        self.get_logger().info(f'Nav2 goal accepted: ({x:.1f}, {y:.1f}) in map frame')
def main():
    p=argparse.ArgumentParser(description='Send a live Nav2 goal in SLAM map coordinates'); p.add_argument('--x',type=float,default=145.0); p.add_argument('--y',type=float,default=0.0); p.add_argument('--yaw',type=float,default=0.0); a=p.parse_args()
    out=Path('results/latest'); out.mkdir(parents=True,exist_ok=True); goal={'x':a.x,'y':a.y,'yaw':a.yaw,'status':'sent','time':time.time()}; (out/'nav_goal.json').write_text(json.dumps(goal))
    rclpy.init(); node=GoalClient()
    try: node.send(a.x,a.y,a.yaw)
    except Exception as e:
        goal.update(status='failed',error=str(e)); (out/'nav_goal.json').write_text(json.dumps(goal)); raise
    finally: node.destroy_node(); rclpy.shutdown()
