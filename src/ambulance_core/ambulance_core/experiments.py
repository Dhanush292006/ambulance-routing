import argparse, csv, json, time
from pathlib import Path
from .demo import run
def main():
    p=argparse.ArgumentParser(); p.add_argument('--scenario',default='research_demo'); a=p.parse_args(); rows=[]
    for planner in ['dijkstra','astar','dstar_lite','dt-aagr']:
        started=time.perf_counter(); twin=run(a.scenario,planner); s=twin.snapshot(); rows.append({'planner':planner,'elapsed_s':time.perf_counter()-started,'distance_m':s['distance_m'],'replans':s['replans'],'battery_pct':s['battery_pct'],'success':s['position']=='HOSPITAL'})
    out=Path('results')/a.scenario; out.mkdir(parents=True,exist_ok=True)
    with (out/'comparison.csv').open('w',newline='') as f: csv.DictWriter(f,fieldnames=rows[0]).writeheader(); csv.DictWriter(f,fieldnames=rows[0]).writerows(rows)
    (out/'summary.json').write_text(json.dumps(rows,indent=2)); print(json.dumps(rows,indent=2))
