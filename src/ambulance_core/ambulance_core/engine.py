from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
import heapq, json, math, time

DEFAULT_ENERGY = {'battery_wh': 4000.0, 'rolling_wh_per_m': 0.042, 'acceleration_wh_per_mps2': 0.12, 'computation_w': 38.0}

def load_energy_config(path=None):
    """Load energy.yaml settings without requiring a YAML runtime dependency."""
    candidates = [Path(path)] if path else [Path(__file__).resolve().parents[3] / 'config' / 'energy.yaml', Path('/etc/ambulance_routing/energy.yaml')]
    for candidate in candidates:
        try:
            lines = candidate.read_text().splitlines()
        except (OSError, TypeError):
            continue
        values = dict(DEFAULT_ENERGY)
        in_energy = False
        for line in lines:
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            if not line[0].isspace():
                in_energy = line.strip() == 'energy:'
            elif in_energy and ':' in line:
                key, raw = line.strip().split(':', 1)
                if key in values:
                    try:
                        value = float(raw.strip())
                        if math.isfinite(value) and value > 0:
                            values[key] = value
                    except ValueError:
                        pass
        return values
    return dict(DEFAULT_ENERGY)

@dataclass
class Edge:
    a: str; b: str; distance: float; speed: float; traffic: float = .2; risk: float = 0.; blocked: bool = False
    def key(self): return f'{self.a}:{self.b}'
    def energy(self, rolling_wh_per_m=.042): return self.distance * rolling_wh_per_m * (1 + .12 * self.traffic + .25 * self.risk)
    def travel_time(self): return self.distance / max(1.0, self.speed * (1 - .55 * self.traffic))

@dataclass
class Obstacle:
    id: str; kind: str; x: float; y: float; vx: float; vy: float; risk: float = .0
    def predict(self, seconds): return (self.x + self.vx * seconds, self.y + self.vy * seconds)

NODES = {'STATION':(0,0),'A':(35,0),'B':(72,0),'C':(108,0),'HOSPITAL':(145,0),'N1':(28,30),'N2':(78,32),'N3':(120,27),'S1':(35,-28),'S2':(85,-30),'S3':(120,-25)}
ROAD_DATA = [('STATION','A',35,10),('A','B',37,11),('B','C',36,11),('C','HOSPITAL',37,10),('STATION','N1',42,9),('N1','N2',50,12),('N2','N3',43,12),('N3','HOSPITAL',38,9),('STATION','S1',45,9),('S1','S2',50,10),('S2','S3',35,10),('S3','HOSPITAL',35,9),('A','N1',30,8),('B','N2',32,8),('C','N3',30,8),('A','S1',28,8),('B','S2',34,8),('C','S3',28,8),('N2','S2',62,10)]

class DigitalTwin:
    def __init__(self, planner='dt-aagr', emergency=True, energy_config=None):
        self.edges=[Edge(*r) for r in ROAD_DATA]; self.planner=planner; self.emergency=emergency
        self.energy_config = load_energy_config() if energy_config is None else {**DEFAULT_ENERGY, **energy_config}
        self.energy_used_wh = 0.
        self.t=0.; self.position='STATION'; self.goal='HOSPITAL'; self.route=[]; self.obstacles=[]; self.events=[]; self.battery=100.; self.distance=0.; self.replans=0; self.last_update_ms=0.; self.affected=[]; self.green_corridor=False; self.speed=0.; self.start_time=time.perf_counter()
        self.plan('initial route')
    def log(self, message): self.events.append({'t':round(self.t,2),'event':message})
    def weights(self): return (.20,.42,.10,.16,.12) if self.emergency else (.34,.23,.20,.13,.10)
    def cost(self,e):
        if self.planner == 'dijkstra':
            return e.distance + (10000 if e.blocked else 0)
        d,t,en,tr,r=self.weights(); return d*e.distance + t*e.travel_time()*10 + en*e.energy(self.energy_config['rolling_wh_per_m'])*10 + tr*e.traffic*100 + r*(e.risk*100 + (10000 if e.blocked else 0))
    def neighbors(self,n):
        for e in self.edges:
            if e.a==n: yield e.b,e
            if e.b==n: yield e.a,Edge(e.b,e.a,e.distance,e.speed,e.traffic,e.risk,e.blocked)
    def shortest(self,start='STATION',goal='HOSPITAL'):
        q=[(0,0,start,[])]; seen={}
        while q:
            _,c,n,path=heapq.heappop(q)
            if c>=seen.get(n,float('inf')): continue
            seen[n]=c
            if n==goal: return path,c
            for nxt,e in self.neighbors(n):
                if not e.blocked:
                    new=c+self.cost(e); h=0.
                    if self.planner == 'astar':
                        x,y=NODES[nxt]; gx,gy=NODES[goal]; h=.2*math.hypot(gx-x,gy-y)
                    # D* Lite mode reuses changed edge values on the persistent twin;
                    # its queue remains local to the current affected subgraph.
                    heapq.heappush(q,(new+h,new,nxt,path+[e]))
        return [],float('inf')
    def plan(self, why, start=None, goal=None):
        if start is not None: self.position=start
        if goal is not None: self.goal=goal
        begin=time.perf_counter(); route,cost=self.shortest(self.position, self.goal); self.last_update_ms=(time.perf_counter()-begin)*1000
        if not route: self.log('NO SAFE ROUTE AVAILABLE'); return
        self.route=route; self.replans+=1; self.log(f'{why}: selected {" → ".join([self.position]+[e.b for e in route])} ({self.last_update_ms:.2f} ms)')
    def update_obstacles(self, obstacles, horizon=5.):
        self.obstacles=obstacles; affected=[]; started=time.perf_counter()
        for e in self.edges:
            ax,ay=NODES[e.a]; bx,by=NODES[e.b]; e.risk=0.; e.blocked=False
            for o in obstacles:
                px,py=o.predict(horizon)
                u=((px-ax)*(bx-ax)+(py-ay)*(by-ay))/max(1,(bx-ax)**2+(by-ay)**2); u=max(0,min(1,u)); dx=px-(ax+u*(bx-ax)); dy=py-(ay+u*(by-ay)); near=math.hypot(dx,dy)
                if near<10:
                    e.risk=max(e.risk,(10-near)/10*o.risk)
                    if o.risk>.8 and near<5: e.blocked=True
                    affected.append(e.key())
        self.affected=sorted(set(affected)); self.last_update_ms=(time.perf_counter()-started)*1000
        if self.affected: self.log(f'prediction affects {len(self.affected)} edges; local update {self.last_update_ms:.2f} ms')
    def account_energy(self, distance_m, dt_s, acceleration_mps2=0., traffic=0., risk=0.):
        """Account measured motion and compute time for an external simulator such as Gazebo."""
        cfg = self.energy_config
        rolling = max(0., distance_m) * cfg['rolling_wh_per_m'] * (1 + .12 * traffic + .25 * risk)
        acceleration = cfg['acceleration_wh_per_mps2'] * max(0., acceleration_mps2)
        computation = cfg['computation_w'] * max(0., dt_s) / 3600
        self.energy_used_wh += rolling + acceleration + computation
        self.battery = max(0., 100. * (1. - self.energy_used_wh / cfg['battery_wh']))

    def tick(self, dt=1.):
        self.t+=dt
        if not self.route: return
        # The model maps graph speed to a conservative urban cruise fraction.
        # This leaves time for the 20 s emergency incident to occur in demo mode.
        e=self.route[0]; previous_speed=self.speed; travel=e.speed*dt*.25; self.speed=e.speed*.25
        self.distance+=travel
        cfg=self.energy_config
        rolling=travel*cfg['rolling_wh_per_m']*(1+.12*e.traffic+.25*e.risk)
        acceleration=cfg['acceleration_wh_per_mps2']*abs(self.speed-previous_speed)
        computation=cfg['computation_w']*dt/3600
        self.energy_used_wh += rolling+acceleration+computation
        self.battery=max(0.,100.*(1.-self.energy_used_wh/cfg['battery_wh']))
        # Consume route edge proportionally using a remaining-distance attribute.
        remaining=getattr(e,'remaining',e.distance)-travel; e.remaining=remaining
        if remaining<=0:
            self.position=e.b; self.route.pop(0); self.log(f'passed {self.position}')
            if self.position=='HOSPITAL': self.log('hospital reached'); self.speed=0.
        if self.emergency and self.route and NODES[self.route[0].a][0]-NODES[self.position][0] < 45: self.green_corridor=True
    def snapshot(self):
        return {'time_s':self.t,'position':self.position,'battery_pct':self.battery,'energy_used_wh':self.energy_used_wh,'energy_config':self.energy_config.copy(),'speed_mps':self.speed,'planner':self.planner,'goal':self.goal,'emergency_active':self.emergency,'green_corridor':self.green_corridor,'route':[e.key() for e in self.route],'obstacles':[asdict(o)|{'predicted':o.predict(5)} for o in self.obstacles],'affected_edges':self.affected,'replans':self.replans,'local_update_ms':self.last_update_ms,'distance_m':self.distance,'events':self.events[-20:]}
    def write_results(self, folder='results/latest'):
        p=Path(folder); p.mkdir(parents=True,exist_ok=True); (p/'metrics.json').write_text(json.dumps(self.snapshot(),indent=2)); (p/'events.json').write_text(json.dumps(self.events,indent=2)); (p/'route_history.json').write_text(json.dumps([asdict(e) for e in self.edges],indent=2))
