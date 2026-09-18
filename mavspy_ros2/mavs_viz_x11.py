#!/usr/bin/env python3
"""
mavs_viz_x11.py  -  live X11 dashboard for the MAVS sim
Shows: topic health, camera image, 3D lidar point cloud,
       top-down map, speed/IMU time series, vehicle state.

Usage:
    apptainer shell --nv \
        -B /scratch/cgoodin/mavs_nav2_sim_ws \
        -B /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY=$DISPLAY \
        /cavs/projects/MAVS/containers/mavs_nav2_sim.sif

    source /opt/ros/humble/setup.bash
    source /scratch/cgoodin/mavs_nav2_sim_ws/install/setup.bash
    python3 mavs_viz_x11.py
"""

import math, struct, json, threading, time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix, Image
from std_msgs.msg import String

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
import numpy as np

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
STALE_S = 2.0   # seconds before a topic is considered stale

state = {
    'gt_x': 0.0, 'gt_y': 0.0,
    'heading_deg': 0.0, 'speed': 0.0,
    'cmd_vx': 0.0, 'cmd_wz': 0.0,
    'lat': 0.0, 'lon': 0.0,
    'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0, 'gyro_z': 0.0,
    'scan_xyz': [],          # list of (x,y,z) in world frame
    'scan_count': 0,
    'trajectory': [],
    'image_rgb': None,       # H x W x 3 uint8 numpy array
    'occ_grid': None,        # occupancy grid dict
    'global_path': [],       # list of (x,y) world coords
    'local_path':  [],       # list of (x,y) world coords
    'ts_speed': [], 'ts_cmd': [],
    'ts_ax': [], 'ts_ay': [], 'ts_az': [],
    # last-received timestamps per topic
    'last': {
        'nature/odometry': 0, 'nature/points': 0, 'nature/cmd_vel': 0,
        'nature/occupancy_grid_vis': 0, 'nature/global_path': 0,
        'nature/local_path': 0, '/camera/image_raw': 0,
    },
}
lock = threading.Lock()
MAX_TS = 200

# Teleop state — modified by keyboard, read by teleop thread
teleop = {
    'throttle': 0.0,   # 0.0 to 1.0  (maps to linear.x m/s * max_speed)
    'steering': 0.0,   # -1.0 to 1.0 (maps to angular.z rad/s)
    'braking':  False,
    'enabled':  True,
    'max_speed': 5.0,
    'max_steer': 0.8,
}
teleop_lock = threading.Lock()
_viz_node_ref = [None]  # set after node creation


def quat_to_yaw(w, x, y, z):
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------
class VizNode(Node):
    def __init__(self):
        super().__init__('mavs_viz_x11')
        # Use sim time to match the running simulation
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
                rclpy.parameter.Parameter.Type.BOOL, True)
        ])
        self._twist_pub = self.create_publisher(Twist, 'nature/cmd_vel', 10)
        self.create_subscription(Odometry,    'nature/odometry',   self._odom, 10)
        self.create_subscription(PointCloud2, 'nature/points',     self._scan, 10)
        self.create_subscription(Image,       '/camera/image_raw', self._img,  10)
        self.create_subscription(Twist,         'nature/cmd_vel',            self._cmd,        10)
        self.create_subscription(OccupancyGrid, 'nature/occupancy_grid_vis', self._occ_grid,   10)
        self.create_subscription(Path,          'nature/global_path',        self._global_path, 10)
        self.create_subscription(Path,          'nature/local_path',         self._local_path,  10)
        self.create_subscription(NavSatFix,          '/gps/fix',         self._gps_fix,  10)

    def _stamp(self, topic):
        with lock:
            state['last'][topic] = time.time()
        # Print first receipt of each topic
        if not hasattr(self, '_seen'):
            self._seen = set()
        if topic not in self._seen:
            self._seen.add(topic)
            print(f'[viz] first message on {topic}')

    def _gps_fix(self, msg):
        with lock:
            state['lat'] = msg.latitude
            state['lon'] = msg.longitude
    
    def _odom(self, msg):
        self._stamp('nature/odometry')
        o = msg.pose.pose.orientation
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        spd = math.sqrt(vx**2 + vy**2)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        with lock:
            state['heading_deg'] = math.degrees(quat_to_yaw(o.w, o.x, o.y, o.z))
            state['speed'] = spd
            state['gt_x'] = x
            state['gt_y'] = y
            state['ts_speed'].append(spd)
            state['ts_cmd'].append(state['cmd_vx'])
            if len(state['ts_speed']) > MAX_TS:
                state['ts_speed'].pop(0)
                state['ts_cmd'].pop(0)
            traj = state['trajectory']
            if not traj or abs(x-traj[-1][0]) > 0.3 or abs(y-traj[-1][1]) > 0.3:
                traj.append((x, y))
                if len(traj) > 600: traj.pop(0)

    def _gt(self, msg):
        pass  # merged into _odom for nature-stack

    def _scan(self, msg):
        self._stamp('nature/points')
        pts = []
        ps = msg.point_step
        try:
            data = bytes(msg.data)
            step = max(1, msg.width // 800)   # downsample to ~800 pts
            with lock:
                vx, vy = state['gt_x'], state['gt_y']
            for i in range(0, msg.width, step):
                x, y, z = struct.unpack_from('fff', data, i * ps)
                if (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
                        and (x-vx)**2 + (y-vy)**2 < 120**2):
                    pts.append((x, y, z))
        except Exception:
            pass
        with lock:
            state['scan_xyz'] = pts
            state['scan_count'] = msg.width

    def _img(self, msg):
        self._stamp('/camera/image_raw')
        try:
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.width, 3))
            with lock:
                state['image_rgb'] = arr.copy()
        except Exception:
            pass

    def _cmd(self, msg):
        self._stamp('nature/cmd_vel')
        with lock:
            state['cmd_vx'] = msg.linear.x   # throttle 0-1
            state['cmd_wz'] = msg.angular.z  # steering rad

    def _radar(self, msg):
        pass  # not used with nature-stack

    def _occ_grid(self, msg):
        self._stamp('nature/occupancy_grid_vis')
        try:
            with lock:
                state['occ_grid'] = {
                    'width':    msg.info.width,
                    'height':   msg.info.height,
                    'res':      msg.info.resolution,
                    'origin_x': msg.info.origin.position.x,
                    'origin_y': msg.info.origin.position.y,
                    'data':     list(msg.data),
                }
        except Exception:
            pass

    def _global_path(self, msg):
        self._stamp('nature/global_path')
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with lock:
            state['global_path'] = pts

    def _local_path(self, msg):
        self._stamp('nature/local_path')
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with lock:
            state['local_path'] = pts

    def _imu(self, msg):
        ax_ = msg.linear_acceleration.x
        ay_ = msg.linear_acceleration.y
        az_ = msg.linear_acceleration.z
        with lock:
            state['accel_x'] = ax_
            state['accel_y'] = ay_
            state['accel_z'] = az_
            state['gyro_z']  = msg.angular_velocity.z
            state['ts_ax'].append(ax_)
            state['ts_ay'].append(ay_)
            state['ts_az'].append(az_)
            if len(state['ts_ax']) > MAX_TS:
                state['ts_ax'].pop(0)
                state['ts_ay'].pop(0)
                state['ts_az'].pop(0)

    def publish_twist(self, throttle, angular_z, braking=0.0):
        # nature/cmd_vel convention: linear.x=throttle, linear.y=braking, angular.z=steering
        msg = Twist()
        msg.linear.x  = float(throttle)
        msg.linear.y  = float(braking)
        msg.angular.z = float(angular_z)
        self._twist_pub.publish(msg)


# ---------------------------------------------------------------------------
# Figure layout
# ---------------------------------------------------------------------------
BG   = '#0d1117'
FG   = '#e6edf3'
GRID = '#1c2128'
BLUE = '#58a6ff'
GRN  = '#3fb950'
RED  = '#f85149'
YLW  = '#d29922'
DIM  = '#8b949e'

plt.rcParams.update({
    'figure.facecolor': BG, 'axes.facecolor': BG,
    'axes.edgecolor': GRID, 'axes.labelcolor': FG,
    'xtick.color': FG, 'ytick.color': FG,
    'text.color': FG, 'grid.color': GRID,
    'grid.alpha': 0.5, 'axes.titlecolor': BLUE,
    'font.family': 'monospace', 'font.size': 9,
})

fig = plt.figure(figsize=(16, 9), facecolor=BG)
fig.canvas.manager.set_window_title('MAVS Sim Monitor')

# Grid: 4 rows x 4 cols with height ratios
# Row 0:   [topic health (col 0-1)] [camera top   (col 2-3)]
# Row 1:   [top-down map (col 0-1)] [camera bottom (col 2-3)]  <- camera spans rows 0-1
# Row 2:   [top-down map (col 0-1)] [occupancy grid (col 2-3)] <- map spans rows 1-2, occ spans 2-3... 
# Simpler: 5 rows, camera spans 0-1, map spans 1-3, occ spans 1-3, teleop row 4
gs = gridspec.GridSpec(5, 4, figure=fig, hspace=0.4, wspace=0.3,
                       left=0.06, right=0.97, top=0.95, bottom=0.04,
                       height_ratios=[0.6, 1, 1, 1, 0.35])

ax_topics = fig.add_subplot(gs[0, 0:2])
ax_cam    = fig.add_subplot(gs[0:2, 2:4])   # camera spans rows 0-1 (tall)
ax_map    = fig.add_subplot(gs[1:4, 0:2])   # map spans rows 1-3
ax_occ    = fig.add_subplot(gs[2:4, 2:4])   # occ spans rows 2-3
ax_teleop = fig.add_subplot(gs[4, 0:4])

for ax in [ax_topics, ax_cam, ax_map, ax_occ]:
    ax.set_facecolor(BG)

ax_occ.grid(True, lw=0.5)
ax_topics.axis('off')
ax_cam.axis('off')
ax_teleop.axis('off')
ax_teleop.set_facecolor(BG)
ax_map.grid(True, lw=0.5)

# --- teleop panel artists ---
# Throttle bar
throttle_bar = ax_teleop.barh([0], [0], color=GRN, height=0.3, left=0)
brake_bar    = ax_teleop.barh([0], [0], color=RED, height=0.3, left=0)
# Steering indicator (vertical bar centered)
steer_bar    = ax_teleop.barh([1], [0], color=YLW, height=0.3, left=0)
teleop_text  = ax_teleop.text(0.5, 0.5, '', transform=ax_teleop.transAxes,
                               ha='center', va='center', fontsize=10,
                               color=FG, family='monospace')
ax_teleop.set_xlim(-1.1, 1.1)
ax_teleop.set_ylim(-0.5, 2.0)

# --- map artists ---
scan_sc2d  = ax_map.scatter([], [], s=2, c=[], cmap='RdYlGn_r',
                             vmin=-1.0, vmax=3.0, alpha=0.65, linewidths=0)
traj_ln,   = ax_map.plot([], [], color=BLUE, lw=1.5, alpha=0.7)
veh_dot,   = ax_map.plot([], [], 'o', color=BLUE, ms=8, zorder=5)
hdg_arr    = ax_map.annotate('', xy=(0,1), xytext=(0,0),
                arrowprops=dict(arrowstyle='->', color=RED, lw=2.5))
ax_map.set_aspect('equal')
ax_map.set_title('Trajectory')
ax_map.set_xlabel('ENU X (m)')
ax_map.set_ylabel('ENU Y (m)')

# --- 3D lidar artist ---
occ_im = ax_occ.imshow(
    np.zeros((10,10), dtype=np.float32),
    origin='lower', cmap='gray_r', vmin=-1, vmax=100,
    extent=[0,1,0,1], aspect='auto', interpolation='nearest',
)
global_path_ln, = ax_occ.plot([], [], '-', color=YLW, lw=2, label='global path')
local_path_ln,  = ax_occ.plot([], [], '-', color=GRN, lw=2, label='local path')
occ_veh_dot,    = ax_occ.plot([], [], '^', color=BLUE, ms=8, zorder=5)
ax_occ.legend(fontsize=7, facecolor=BG, labelcolor=FG, loc='upper right')
ax_occ.set_title('Occupancy grid + paths')
ax_occ.set_xlabel('X (m)')
ax_occ.set_ylabel('Y (m)')

# --- camera image artist ---
#cam_im = ax_cam.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
cam_im = ax_cam.imshow(np.zeros((960, 720, 3), dtype=np.uint8))
ax_cam.set_title('Camera')
ax_cam.set_xticks([]); ax_cam.set_yticks([])

# Speed / state as text — placed inside the map axes data space
# We use ax_map data coords updated each frame so it moves with the map view
map_info_text = ax_map.text(0, 0, '', ha='left', va='bottom',
    fontsize=9, color=FG, family='monospace', zorder=10,
    bbox=dict(facecolor=BG, alpha=0.85, edgecolor=GRID, linewidth=1))

# topic status boxes
TOPICS = [
    'nature/odometry', 'nature/points', 'nature/cmd_vel',
    'nature/occupancy_grid_vis', 'nature/global_path', 'nature/local_path',
    '/camera/image_raw',
]
topic_boxes = []
topic_labels = []
# Fit all topics in one row across the full width
n = len(TOPICS)
box_w = 0.96 / n
for i, topic in enumerate(TOPICS):
    x = 0.02 + i * box_w
    y = 0.15
    box = plt.Rectangle((x, y), box_w - 0.01, 0.65,
                         transform=ax_topics.transAxes,
                         facecolor='#21262d', edgecolor=GRID, lw=1)
    ax_topics.add_patch(box)
    topic_boxes.append(box)
    # Show just the last segment of the topic name to fit in the box
    short = topic.split('/')[-1]
    short = short[:12] + '..' if len(short) > 12 else short
    lbl = ax_topics.text(x + (box_w-0.01)/2, y + 0.33, short,
                          transform=ax_topics.transAxes,
                          ha='center', va='center', fontsize=6.5, color=FG)
    topic_labels.append(lbl)

ax_topics.set_title('Topic health')


# ---------------------------------------------------------------------------
# Animation update
# ---------------------------------------------------------------------------
def update(_frame):
    now = time.time()
    with lock:
        gt_x    = state['gt_x']
        gt_y    = state['gt_y']
        hdg     = state['heading_deg']
        spd     = state['speed']
        cmd_vx  = state['cmd_vx']
        cmd_wz  = state['cmd_wz']
        lat     = state['lat']
        lon     = state['lon']
        ax_     = state['accel_x']
        ay_     = state['accel_y']
        az_     = state['accel_z']
        gz      = state['gyro_z']
        scan    = list(state['scan_xyz'])
        traj    = list(state['trajectory'])
        img     = state['image_rgb']
        # ts_speed/imu kept in state but no longer plotted
        sc      = state['scan_count']
        lasts   = dict(state['last'])

    # --- topic health ---
    for i, (topic, box, lbl) in enumerate(zip(TOPICS, topic_boxes, topic_labels)):
        age = now - lasts.get(topic, 0)
        if age < STALE_S:
            color = GRN
            status = f'{age*1000:.0f}ms'
        elif lasts.get(topic, 0) == 0:
            color = DIM
            status = 'no data'
        else:
            color = RED
            status = f'stale {age:.1f}s'
        box.set_edgecolor(color)
        box.set_linewidth(2)
        short = topic.split('/')[-1]
        short = short[:12] + '..' if len(short) > 12 else short
        lbl.set_text(f'{short}\n{status}')
        lbl.set_color(color)

    # --- camera ---
    if img is not None:
        cam_im.set_data(img)
        cam_im.set_extent([0, img.shape[1], img.shape[0], 0])
        #ax_cam.set_title(f'Camera  {img.shape[1]}x{img.shape[0]}')
        ax_cam.set_title(f'Debug Camera')

    # --- top-down map ---
    if scan:
        # Points already in world frame, color by height (z)
        sx = np.array([p[0] for p in scan])
        sy = np.array([p[1] for p in scan])
        sz = np.array([p[2] for p in scan])
        scan_sc2d.set_offsets(np.c_[sx, sy])
        scan_sc2d.set_array(sz)
        scan_sc2d.set_clim(vmin=sz.min(), vmax=sz.max())
    if traj:
        tx, ty = zip(*traj)
        traj_ln.set_data(tx, ty)
    veh_dot.set_data([gt_x], [gt_y])
    hdg_r = math.radians(hdg)
    arr_len = 4
    hdg_arr.set_position((gt_x, gt_y))
    hdg_arr.xy = (gt_x + arr_len * math.cos(hdg_r),
                  gt_y + arr_len * math.sin(hdg_r))
    pad = max(25, max((max(abs(p[0]-gt_x), abs(p[1]-gt_y)) for p in traj),
                      default=25) * 0.75) if traj else 25
    ax_map.set_xlim(gt_x - pad, gt_x + pad)
    ax_map.set_ylim(gt_y - pad, gt_y + pad)
    ax_map.set_title(f'Trajectory  hdg={hdg:.1f}°  {sc} pts  '
                     f'({gt_x:.1f}, {gt_y:.1f})')

    # --- occupancy grid + paths ---
    with lock:
        occ   = state['occ_grid']
        gpath = list(state['global_path'])
        lpath = list(state['local_path'])

    if occ is not None:
        w, h   = occ['width'], occ['height']
        res    = occ['res']
        ox, oy = occ['origin_x'], occ['origin_y']
        arr = np.array(occ['data'], dtype=np.float32).reshape(h, w)
        occ_im.set_data(arr)
        occ_im.set_extent([ox, ox+w*res, oy, oy+h*res])
        ax_occ.set_xlim(ox, ox+w*res)
        ax_occ.set_ylim(oy, oy+h*res)

    global_path_ln.set_data(
        [p[0] for p in gpath], [p[1] for p in gpath])
    local_path_ln.set_data(
        [p[0] for p in lpath], [p[1] for p in lpath])
    occ_veh_dot.set_data([gt_x], [gt_y])
    ax_occ.set_title(
        f'Occupancy grid  global: {len(gpath)} pts  local: {len(lpath)} pts')

    # --- speed / state text — anchor to bottom-left of current map view ---
    xlim = ax_map.get_xlim()
    ylim = ax_map.get_ylim()
    map_info_text.set_position((xlim[0] + (xlim[1]-xlim[0])*0.02,
                                 ylim[0] + (ylim[1]-ylim[0])*0.02))
    map_info_text.set_text(
        f'Speed:   {spd:.2f} m/s\n'
        f'Heading: {hdg:.1f} deg\n'
        f'Cmd thr: {cmd_vx:.2f}  steer: {cmd_wz:.3f}\n'
        f'ENU:     ({gt_x:.1f}, {gt_y:.1f})'
    )

    fig.suptitle(
        f'MAVS SIM MONITOR   |   '
        f'lat={lat:.5f}  lon={lon:.5f}   |   '
        f'ENU ({gt_x:.1f}, {gt_y:.1f})',
        color=FG, fontsize=10, y=0.99
    )

    # --- teleop panel ---
    with teleop_lock:
        thr  = teleop['throttle']
        brk  = teleop['braking']
        steer = teleop['steering']
        enabled = teleop['enabled']

    # Throttle/brake bar
    throttle_bar[0].set_width(thr if not brk else 0)
    brake_bar[0].set_width(-0.3 if brk else 0)
    # Steering bar centered at 0
    steer_bar[0].set_x(min(0, steer))
    steer_bar[0].set_width(abs(steer))

    mode = 'TELEOP' if enabled else 'NAV2'
    teleop_text.set_text(
        f'[{mode}]  '
        f'Throttle: {thr:+.2f}  '
        f'Steer: {steer:+.2f}  '
        f'Brake: {"ON" if brk else "off"}   '
        f'  |  Arrow keys: drive   Space: brake   T: toggle teleop/autonomy'
    )
    teleop_text.set_color(GRN if enabled else DIM)


# ---------------------------------------------------------------------------
# Keyboard teleop
# ---------------------------------------------------------------------------
def on_key_press(event):
    with teleop_lock:
        k = event.key
        if k == 't':
            teleop['enabled'] = not teleop['enabled']
            if not teleop['enabled']:
                # Stop vehicle when switching to nav2 mode
                teleop['throttle'] = 0.0
                teleop['steering'] = 0.0
                teleop['braking']  = False
        if not teleop['enabled']:
            return
        if k == 'up':
            teleop['throttle'] = min(1.0, teleop['throttle'] + 0.1)
            teleop['braking']  = False
        elif k == 'down':
            teleop['throttle'] = max(-1.0, teleop['throttle'] - 0.1)
            teleop['braking']  = False
        elif k == 'left':
            teleop['steering'] = min(1.0, teleop['steering'] + 0.1)
        elif k == 'right':
            teleop['steering'] = max(-1.0, teleop['steering'] - 0.1)
        elif k == ' ':
            teleop['braking']  = True
            teleop['throttle'] = 0.0
        elif k == 'x':
            # Full stop
            teleop['throttle'] = 0.0
            teleop['steering'] = 0.0
            teleop['braking']  = False

def on_key_release(event):
    with teleop_lock:
        if not teleop['enabled']:
            return
        k = event.key
        if k == ' ':
            teleop['braking'] = False
        # Auto-center steering on release
        elif k in ('left', 'right'):
            teleop['steering'] *= 0.5  # decay toward center

def teleop_publish_loop():
    """Publishes cmd_vel at 20Hz from teleop state."""
    import time
    while True:
        node = _viz_node_ref[0]
        if node is not None:
            with teleop_lock:
                enabled = teleop['enabled']
                thr     = teleop['throttle']
                steer   = teleop['steering']
                brk     = teleop['braking']
                max_spd = teleop['max_speed']
                max_st  = teleop['max_steer']
            if enabled:
                # nature expects throttle (0-1), not speed
                throttle  = 0.0 if brk else max(0.0, thr)
                braking   = 1.0 if brk else (max(0.0, -thr))
                angular_z = steer * max_st
                node.publish_twist(throttle, angular_z, braking)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def ros_thread():
    rclpy.init()
    node = VizNode()
    _viz_node_ref[0] = node
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    threading.Thread(target=ros_thread, daemon=True).start()
    threading.Thread(target=teleop_publish_loop, daemon=True).start()
    # Give ROS a moment to connect
    time.sleep(1.0)
    # Wire up keyboard events
    fig.canvas.mpl_connect('key_press_event',   on_key_press)
    fig.canvas.mpl_connect('key_release_event', on_key_release)
    anim = FuncAnimation(fig, update, interval=250,
                         blit=False, cache_frame_data=False)
    plt.show()
