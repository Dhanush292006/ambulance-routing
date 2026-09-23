"""Live Streamlit control-room view; polls ROS-derived files every two seconds."""
from pathlib import Path
import json
import numpy as np
import streamlit as st

st.set_page_config(page_title='Smart Emergency Response', layout='wide')
st.title('SMART EMERGENCY RESPONSE SYSTEM')
st.caption('● SYSTEM ONLINE — LIVE DIGITAL TWIN CONTROL CENTER')

@st.fragment(run_every='2s')
def live_control_room():
    path=Path('results/latest/metrics.json')
    if not path.exists():
        st.info('Waiting for the digital-twin ROS node. Start `./gazebo.sh` or `./run.sh --demo`.')
        return
    d=json.loads(path.read_text())
    a,b,c,dcol=st.columns(4)
    a.metric('AMBULANCE',d['position']); b.metric('Battery',f"{d['battery_pct']:.1f}%")
    c.metric('Speed',f"{d['speed_mps']*3.6:.1f} km/h"); dcol.metric('Planner',d['planner'])
    goal_path=Path('results/latest/nav_goal.json')
    if goal_path.exists():
        g=json.loads(goal_path.read_text())
        st.info(f"NAV2 GOAL: ({g['x']:.1f}, {g['y']:.1f}) · {g['status'].upper()}")
    o1,o2,o3=st.columns(3)
    o1.metric('LiDAR obstacles',len(d['obstacles'])); o2.metric('Affected graph edges',len(d['affected_edges']))
    o3.metric('Local graph update',f"{d['local_update_ms']:.2f} ms")
    st.success('AMBULANCE GREEN CORRIDOR ACTIVE' if d['green_corridor'] else 'Green corridor standing by')
    left,right=st.columns([2,1])
    with left:
        st.subheader('LIVE SLAM / DIGITAL TWIN MAP')
        slam_path=Path('results/latest/slam_map.json')
        if slam_path.exists():
            m=json.loads(slam_path.read_text()); grid=np.asarray(m['data'],dtype=np.int16).reshape(m['height'],m['width'])
            image=np.where(grid < 0,190,np.where(grid > 50,20,255)).astype(np.uint8)
            st.image(image,caption=f"LIVE /map — {m['width']}×{m['height']}, {m['resolution']:.3f} m/cell",clamp=True)
        else: st.info('Waiting for SLAM map. Teleoperate the ambulance to scan the city.')
        st.caption('Route: ' + (' → '.join(d['route']) if d['route'] else 'hospital reached / no active route'))
    with right:
        st.subheader('GRAPH REPLANNING')
        st.metric('Replans',d['replans']); st.metric('Distance travelled',f"{d['distance_m']:.1f} m")
        st.write('Affected edges:', ', '.join(d['affected_edges']) if d['affected_edges'] else 'none')
    st.subheader('LIVE OBSTACLE AVOIDANCE')
    if d['obstacles']:
        rows=[]
        for x in d['obstacles']:
            px,py=x['predicted']; rows.append({'ID':x['id'],'Type':x['kind'],'Position':f"({x['x']:.1f}, {x['y']:.1f}) m",'Predicted':f"({px:.1f}, {py:.1f}) m",'Risk':f"{x['risk']:.2f}"})
        st.warning('OBSTACLE DETECTED — Localized route update active'); st.dataframe(rows,use_container_width=True,hide_index=True)
    else: st.success('CLEAR CORRIDOR — No LiDAR road obstacle currently detected')
    st.subheader('LIVE EVENT LOG'); st.dataframe(d['events'],use_container_width=True,hide_index=True)
    st.caption('Live server refresh: 2 seconds · inputs: ROS Digital Twin + SLAM Toolbox + Gazebo sensor bridge')

live_control_room()
