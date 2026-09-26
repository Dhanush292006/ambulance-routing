"""ROS 2 mission control and live city digital-twin dashboard."""
from pathlib import Path
import json
import math
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw
import streamlit as st

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ambulance_core'))
from ambulance_core.engine import NODES

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / 'results/latest'
HOSPITALS = {
    'Main Emergency Hospital': 'HOSPITAL-01',
    'Government Hospital': 'HOSPITAL-02',
    'Trauma Centre': 'HOSPITAL-03',
    'Multi-Speciality Hospital': 'HOSPITAL-04',
    'City Medical Centre': 'HOSPITAL-05',
}

st.set_page_config(page_title='Smart Emergency Response', layout='wide')
st.title('SMART EMERGENCY RESPONSE SYSTEM')
st.caption('ROS 2 / NAV2 MISSION CONTROL · CHENNAI CITY DIGITAL TWIN · LIVE REFRESH EVERY 2 SEC')


def read_json(name):
    try:
        return json.loads((RESULTS / name).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def send_command(action, destination_id=None):
    payload = json.dumps({'action': action, **({'destination_id': destination_id} if destination_id else {})})
    yaml_payload = "{data: '" + payload.replace("'", "''") + "'}"
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'pub', '--once', '/dashboard/mission_command',
             'std_msgs/msg/String', yaml_payload], cwd=ROOT, capture_output=True,
            text=True, timeout=8, check=False,
        )
        if result.returncode:
            st.error('ROS 2 command failed: ' + (result.stderr.strip() or result.stdout.strip()))
        else:
            st.toast(f'{action.upper()} command sent to ROS 2')
    except (OSError, subprocess.TimeoutExpired) as exc:
        st.error(f'Cannot reach ROS 2 CLI. Start Gazebo, ROS, and Nav2 first. {exc}')


def draw_vehicle_model(pose, status, metrics):
    """Render a small top-down ambulance view rotated from live odometry."""
    width, height = 560, 300
    image = Image.new('RGB', (width, height), (13, 25, 38))
    draw = ImageDraw.Draw(image)
    for x in range(0, width, 32):
        draw.line((x, 0, x, height), fill=(24, 40, 53), width=1)
    for y in range(0, height, 32):
        draw.line((0, y, width, y), fill=(24, 40, 53), width=1)
    draw.rectangle((20, 20, width - 20, height - 20), outline=(58, 84, 99), width=2)
    draw.text((34, 30), 'AMBULANCE 01  ·  LIVE DIGITAL TWIN', fill=(220, 237, 245))
    draw.text((34, 53), 'TOP VIEW  /  ODOMETRY-DRIVEN MODEL', fill=(117, 165, 184))

    cx, cy = 280, 170
    yaw = float((pose or {}).get('yaw', 0.0))
    c, s = math.cos(yaw), math.sin(yaw)

    def xy(x, y):
        return (cx + x * c - y * s, cy - (x * s + y * c))

    def box(cx_local, cy_local, length, breadth, color, outline=None):
        local = [(-length/2, -breadth/2), (length/2, -breadth/2),
                 (length/2, breadth/2), (-length/2, breadth/2)]
        points = [xy(cx_local + x, cy_local + y) for x, y in local]
        draw.polygon(points, fill=color, outline=outline or color)

    # Wheels sit just outside the ambulance body.
    for wx in (-66, 66):
        for wy in (-51, 51):
            box(wx, wy, 42, 16, (8, 12, 17), (104, 120, 130))
    box(0, 0, 176, 88, (235, 238, 231), (255, 255, 255))
    box(-12, 0, 80, 72, (40, 83, 107), (87, 150, 177))
    box(55, 0, 42, 76, (219, 224, 219), (255, 255, 255))
    box(35, 0, 9, 72, (206, 35, 36))
    box(35, 0, 38, 9, (206, 35, 36))
    box(-83, 0, 15, 54, (22, 87, 221))
    # Nose marker and heading line show actual yaw.
    nose = xy(108, 0)
    draw.ellipse((nose[0]-7, nose[1]-7, nose[0]+7, nose[1]+7), fill=(60, 245, 150))
    heading = xy(145, 0)
    draw.line((cx, cy, heading[0], heading[1]), fill=(60, 245, 150), width=3)

    x = (pose or {}).get('x')
    y = (pose or {}).get('y')
    speed = float((metrics or {}).get('speed_mps', (status or {}).get('speed_mps', 0.0)) or 0.0)
    if x is None or y is None:
        pose_text = 'Awaiting Gazebo odometry'
    else:
        pose_text = f'World pose  X {x:+.2f} m   Y {y:+.2f} m   Yaw {math.degrees(yaw):+.0f}°'
    draw.text((34, 258), pose_text, fill=(202, 222, 231))
    draw.text((365, 258), f'{speed:.2f} m/s', fill=(80, 232, 164))
    return image


def draw_ambulance_icon(draw, x, y, yaw, scale=1.0):
    """Draw a rotated ambulance marker in occupancy-grid pixel coordinates."""
    length, breadth = 16 * scale, 8 * scale
    c, s = math.cos(yaw), math.sin(yaw)
    local = [(-length/2, -breadth/2), (length/2, -breadth/2),
             (length/2, breadth/2), (-length/2, breadth/2)]
    points = [(x + a*c-b*s, y - (a*s+b*c)) for a, b in local]
    draw.polygon(points, fill=(213, 38, 42), outline=(255, 255, 255))
    nose = (x + length/2*c, y - length/2*s)
    draw.ellipse((nose[0]-1.8, nose[1]-1.8, nose[0]+1.8, nose[1]+1.8), fill=(37, 237, 142))
    cross_x, cross_y = x - length*0.12*c, y + length*0.12*s
    draw.ellipse((cross_x-2.2, cross_y-2.2, cross_x+2.2, cross_y+2.2), fill=(255, 255, 255))


def get_route_nodes(status, metrics):
    nodes = (status or {}).get('route_nodes') or []
    if nodes:
        start = NODES.get('STATION')
        current = (status or {}).get('pose') or {}
        if current.get('x') is not None:
            start = min(NODES.values(), key=lambda p: math.hypot(p[0]-current['x'], p[1]-current['y']))
        return [start] + [NODES[n] for n in nodes if n in NODES]
    edges = (metrics or {}).get('route') or []
    names = []
    for edge in edges:
        if ':' in edge:
            if not names:
                names.append(edge.split(':', 1)[0])
            names.append(edge.split(':', 1)[1])
    return [NODES[n] for n in names if n in NODES]


def make_live_map(status, metrics, slam):
    grid = np.asarray(slam['data'], dtype=np.int16).reshape(slam['height'], slam['width'])
    gray = np.flipud(np.where(grid < 0, 190, np.where(grid > 50, 26, 246))).astype(np.uint8)
    canvas = Image.fromarray(np.stack([gray, gray, gray], axis=-1))
    draw = ImageDraw.Draw(canvas)
    ox, oy = slam['origin']
    res = slam['resolution']

    def pixel(x, y):
        return ((x - ox) / res, slam['height'] - 1 - (y - oy) / res)

    route_pts = [pixel(x, y) for x, y in get_route_nodes(status, metrics)]
    if len(route_pts) > 1:
        draw.line(route_pts, fill=(22, 156, 247), width=max(3, int(2/res)))
    for name, color in [('STATION', (22, 210, 104)), ('HOSPITAL', (239, 51, 53))]:
        px, py = pixel(*NODES[name])
        draw.ellipse((px-7, py-7, px+7, py+7), fill=color, outline=(255, 255, 255), width=2)

    pose = ((status or {}).get('pose') or (metrics or {}).get('pose') or {})
    if pose.get('x') is not None and pose.get('y') is not None:
        mx, my = pixel(float(pose['x']), float(pose['y']))
        draw_ambulance_icon(draw, mx, my, float(pose.get('yaw', 0.0)), scale=1.0)
    else:
        mx, my = pixel(*NODES['STATION'])
        draw_ambulance_icon(draw, mx, my, 0.0, scale=1.0)

    obstacles = (metrics or {}).get('obstacles') or []
    for obstacle in obstacles:
        if obstacle.get('x') is None or obstacle.get('y') is None:
            continue
        px, py = pixel(obstacle['x'], obstacle['y'])
        draw.ellipse((px-5, py-5, px+5, py+5), fill=(255, 151, 32), outline=(255, 255, 255))

    # Focus the live Nav2 corridor so the ambulance stays legible; offer the full map below.
    left, top = pixel(-45, 115)
    right, bottom = pixel(195, -115)
    crop = canvas.crop((int(left), int(top), int(right), int(bottom)))
    return crop, canvas


@st.fragment(run_every='2s')
def mission_panel():
    status = read_json('mission_status.json') or {}
    metrics = read_json('metrics.json') or {}
    slam = read_json('slam_map.json')
    pose = status.get('pose') or metrics.get('pose') or {}
    st.subheader('EMERGENCY MISSION CONTROL')
    chosen = st.selectbox('DESTINATION', list(HOSPITALS), key='hospital')
    cols = st.columns(5)
    if cols[0].button('START MISSION', type='primary', width='stretch'):
        send_command('start', HOSPITALS[chosen])
    if cols[1].button('PAUSE', width='stretch'):
        send_command('pause')
    if cols[2].button('RESUME', width='stretch'):
        send_command('resume')
    if cols[3].button('CANCEL', width='stretch'):
        send_command('cancel')
    if cols[4].button('RETURN TO STATION', width='stretch'):
        send_command('return')

    rviz_col, route_col = st.columns(2)
    if rviz_col.button('VIEW RVIZ2'):
        try:
            proc = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', str(ROOT / 'rviz/ambulance.rviz')], cwd=ROOT)
            st.session_state['rviz_pid'] = proc.pid
            st.success(f'RViz2 launched (PID {proc.pid})')
        except OSError as exc:
            st.error(f'Could not launch RViz2: {exc}')
    show_route = route_col.toggle('SHOW ROUTE AND TWIN LOG', value=True)

    top = st.columns(5)
    top[0].metric('MISSION', status.get('state', 'IDLE'))
    top[1].metric('TWIN PLANNER', metrics.get('planner', 'DT-AAGR').upper())
    top[2].metric('AMBULANCE SPEED', f"{float(metrics.get('speed_mps', status.get('speed_mps', 0)) or 0) * 3.6:.1f} km/h")
    top[3].metric('BATTERY', f"{float(metrics.get('battery_pct', 100) or 0):.1f}%")
    top[4].metric('DT-AAGR REPLANS', int(metrics.get('replans', status.get('replans', 0)) or 0))

    if pose.get('x') is None:
        st.warning('Waiting for the Gazebo ambulance odometry publisher. The map is centered at the ambulance station until its first pose arrives.')
    else:
        st.caption(f"Live Gazebo pose: X {pose['x']:.2f} m · Y {pose['y']:.2f} m · heading {math.degrees(pose.get('yaw', 0.0)):.0f}°")
    if status.get('destination'):
        destination = status['destination']
        st.caption(f"Destination: {destination.get('name', 'unknown')} · current waypoint: {status.get('current_waypoint') or 'route planning'} · ETA {float(status.get('eta_s', metrics.get('eta_s', 0)) or 0):.0f} sec")
    if status.get('error'):
        st.error(status['error'])

    left, right = st.columns([1.35, 1])
    with left:
        st.subheader('LIVE CITY GRID · AMBULANCE POSITION AND NAV2 ROUTE')
        if slam:
            local_map, full_map = make_live_map(status, metrics, slam)
            st.image(local_map, caption='Live road grid · red ambulance · blue DT-AAGR / Nav2 route · green station · red hospital · orange LiDAR obstacles', width='stretch')
            with st.expander('Full Chennai occupancy grid'):
                st.image(full_map, caption=f"{slam['width']} × {slam['height']} cells · {slam['resolution']:.2f} m/cell · origin {slam['origin']}", width='stretch')
        else:
            st.info('Waiting for the live Nav2 occupancy grid on /map.')

    with right:
        st.subheader('AMBULANCE DIGITAL TWIN · LIVE MODEL WINDOW')
        model_view = draw_vehicle_model(pose, status, metrics)
        st.image(model_view, width='stretch')
        a, b, c = st.columns(3)
        a.metric('ODOMETRY', 'LIVE' if pose.get('x') is not None else 'WAITING')
        b.metric('LIDAR OBJECTS', len(metrics.get('obstacles') or []))
        c.metric('UPDATE COST', f"{float(metrics.get('local_update_ms', 0) or 0):.2f} ms")
        destination = status.get('destination') or {}
        st.caption(f"State: {status.get('state', 'IDLE')} · Goal: {destination.get('name', metrics.get('goal', 'HOSPITAL'))}")
        st.caption('Pose, heading, speed, route, and energy are sourced from the ROS / Gazebo twin.')

    route_nodes = status.get('route_nodes') or []
    if show_route:
        st.subheader('DT-AAGR ROUTE AND DIGITAL-TWIN EVENT LOG')
        route_names = ['STATION'] + route_nodes if route_nodes else [edge.replace(':', ' → ') for edge in metrics.get('route', [])]
        st.write('**Active route:**', ' → '.join(route_names) if route_names else 'Awaiting route plan')
        st.write(f"**Distance travelled:** {float(metrics.get('distance_m', status.get('distance_m', 0)) or 0):.1f} m · **Affected road edges:** {', '.join(metrics.get('affected_edges', [])) or 'none'}")
        events = read_json('events.json') or metrics.get('events') or []
        lines = []
        for event in events[-14:]:
            when = f"t+{float(event.get('t', 0)):.0f}s" if event.get('t') is not None else '--:--:--'
            lines.append(f"{when}  [DT-AAGR]  {event.get('event', '')}")
        if status.get('updated_at'):
            lines.append(f"LIVE  [MISSION]  {status.get('state', 'IDLE')} · waypoint {status.get('current_waypoint') or 'none'} · replans {status.get('replans', 0)}")
        st.code('\n'.join(lines[-15:]) if lines else 'Waiting for digital-twin route and odometry events…', language='text')

        result = read_json('mission_result.json')
        if result:
            with st.expander('Last completed mission'):
                st.json(result, expanded=False)

    with st.expander('LiDAR obstacles and affected road edges'):
        obstacles = metrics.get('obstacles') or []
        if obstacles:
            rows = [{'ID': o.get('id'), 'Type': o.get('kind'), 'Position (m)': f"({o.get('x', 0):.1f}, {o.get('y', 0):.1f})", 'Risk': o.get('risk')} for o in obstacles]
            st.dataframe(rows, width='stretch', hide_index=True)
        else:
            st.write('No obstacles currently reported by the digital twin.')


mission_panel()
