"""Streamlit view of measurements written by digital_twin_node or demo.py."""
from pathlib import Path
import json
import streamlit as st
st.set_page_config(page_title='Smart Emergency Response',layout='wide')
st.title('SMART EMERGENCY RESPONSE SYSTEM')
st.caption('● SYSTEM ONLINE — DIGITAL TWIN CONTROL CENTER')
path=Path('results/latest/metrics.json')
if not path.exists(): st.info('Start `./run.sh --demo` or the ROS launch to populate live telemetry.'); st.stop()
d=json.loads(path.read_text()); a,b,c,dcol=st.columns(4)
a.metric('AMBULANCE', d['position']); b.metric('Battery',f"{d['battery_pct']:.1f}%"); c.metric('Speed',f"{d['speed_mps']*3.6:.1f} km/h"); dcol.metric('Planner',d['planner'])
st.success('AMBULANCE GREEN CORRIDOR ACTIVE' if d['green_corridor'] else 'Green corridor standing by')
left,right=st.columns([2,1])
with left:
    st.subheader('DIGITAL TWIN MAP')
    st.json({'current_route':d['route'],'affected_edges':d['affected_edges'],'obstacles':d['obstacles']})
with right:
    st.subheader('GRAPH REPLANNING')
    st.metric('Localized update',f"{d['local_update_ms']:.2f} ms"); st.metric('Replans',d['replans']); st.metric('Distance travelled',f"{d['distance_m']:.1f} m")
st.subheader('LIVE EVENT LOG'); st.dataframe(d['events'],use_container_width=True)
st.caption('Refresh the page to read the latest simulation measurement file.')
