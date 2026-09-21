import argparse, time
from .engine import DigitalTwin, Obstacle
def run(scenario='research_demo', planner='dt-aagr', realtime=False):
    t=DigitalTwin(planner); t.log('ambulance emergency activated')
    last_signature = None
    for step in range(240):
        obstacles=[]
        if scenario in ('research_demo','road_blocked','accident') and t.t >= 20: obstacles.append(Obstacle('vehicle-01','stopped vehicle',90,0,0,0,1.0))
        if scenario in ('research_demo','pedestrian','dynamic_obstacles') and t.t >= 48: obstacles.append(Obstacle('pedestrian-03','pedestrian',112,27,-.4,0,0.9))
        if scenario in ('heavy_traffic','high_dynamic_traffic'): obstacles.append(Obstacle('bus-01','bus',75,32,.5,0,.55))
        signature = tuple((o.id, o.x, o.y, o.vx, o.vy) for o in obstacles)
        if signature != last_signature:
            t.update_obstacles(obstacles)
            if obstacles: t.plan('localized graph replan')
            last_signature = signature
        t.tick(.5)
        if realtime: time.sleep(.1)
        if t.position=='HOSPITAL': break
    t.write_results(); return t
def main():
    p=argparse.ArgumentParser(); p.add_argument('--scenario',default='research_demo'); p.add_argument('--planner',default='dt-aagr'); p.add_argument('--realtime',action='store_true'); a=p.parse_args(); twin=run(a.scenario,a.planner,a.realtime); print(twin.snapshot())
if __name__=='__main__': main()
