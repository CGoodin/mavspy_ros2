#!/usr/bin/env python3
"""
mavs_sim_node.py
----------------
Drop-in MAVS simulator node for the nature-stack autonomy stack.
Replaces the C++ nature_sim_test_node with a full physics simulation
using mavspy (MAVS Python bindings).

Subscribes
  nature/cmd_vel          geometry_msgs/Twist
                            linear.x  = throttle  (0 to 1)
                            linear.y  = braking    (0 to 1)
                            angular.z = steering   (rad, + = left)

Publishes
  nature/odometry         nav_msgs/Odometry        – vehicle pose/velocity
  nature/points           sensor_msgs/PointCloud2  – lidar in world (odom) frame
  nature/veh              std_msgs/Float64MultiArray – [0,x,y,vx,vy,0,0,0,0]

TF
  odom -> base_link  (dynamic, from vehicle physics)

Usage
-----
  # Clone into your ROS 2 workspace alongside the nature package:
  cp mavs_sim_node.py ~/ros2_ws/src/nature/src/simulation/
  # Then launch nature with this node instead of nature_sim_test_node:
  ros2 run nature mavs_sim_node [--ros-args -p scene_file:=... -p vehicle_file:=...]

  # Or add to your launch file:
  Node(
      package='nature',
      executable='mavs_sim_node',
      name='mavs_sim_node',
      parameters=[{
          'scene_file': '/path/to/scene.json',
          'vehicle_file': '/path/to/vehicle.json',
          'lidar_model': 'VLP-16',
          'use_sim_time': False,
      }],
  )

Parameters
----------
  scene_file        str   – MAVS scene JSON (default: flat terrain)
  vehicle_file      str   – MAVS vehicle JSON (default: Forester)
  lidar_model       str   – MAVS lidar model (default: VLP-16)
  init_x/y/z        float – initial ENU position (m)
  init_heading       float – initial heading (rad)
  dt                float – physics timestep (s, default 0.01)
  lidar_rate        float – lidar publish Hz (default 10.0)
  odom_rate         float – odometry publish Hz (default 100.0)
  lidar_height      float – lidar mount height above base (m, default 1.5)
  desired_speed     float – target speed in m/s passed to throttle controller
                            (set to 0 to use raw throttle from cmd_vel)
"""

import math
import struct
import threading
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Image, NavSatFix, NavSatStatus
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import tf2_ros

# ---------------------------------------------------------------------------
# mavspy import
# ---------------------------------------------------------------------------
import os as _os, subprocess as _sp, sys as _sys

def enu_to_latlon(e, n, u, lat0_deg, lon0_deg, alt0):
    """Simple flat-earth conversion from ENU to WGS-84."""
    # Earth radius for flat-earth ENU->LatLon
    _R_EARTH = 6378137.0  # metres
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    lat  = lat0 + (n / _R_EARTH)
    lon  = lon0 + (e / (_R_EARTH * math.cos(lat0)))
    alt  = alt0 + u
    return math.degrees(lat), math.degrees(lon), alt

def _ensure_mavs_config():
    cwd_cfg  = 'mavs_config.txt'
    home_cfg = _os.path.join(_os.path.expanduser('~'), 'mavs_config.txt')
    try:
        if _os.path.exists(cwd_cfg):
            with open(cwd_cfg) as f:
                if f.read().strip():
                    return
    except Exception:
        pass
    data_path = ''
    try:
        with open(home_cfg) as f:
            data_path = f.read().strip()
    except Exception:
        pass
    if not data_path:
        try:
            r = _sp.run([_sys.executable, '-c',
                         'import mavspy, os; '
                         'print(os.path.dirname(mavspy.__file__)+"/data", end="")'],
                        capture_output=True, text=True, timeout=5)
            if r.returncode == 0:
                data_path = r.stdout.strip()
        except Exception:
            pass
    if data_path:
        for path in [home_cfg, cwd_cfg]:
            try:
                with open(path, 'w') as f:
                    f.write(data_path)
            except Exception:
                pass

_ensure_mavs_config()
import mavspy.mavs as mavs

# ---------------------------------------------------------------------------
# PointCloud2 helpers
# ---------------------------------------------------------------------------
_PC2_FIELDS = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='i', offset=12, datatype=PointField.FLOAT32, count=1),
]
_PT_STEP = 16


def _make_pc2(points_xyzi, frame_id, stamp):
    """Build a PointCloud2 from a list of [x,y,z,i] in world frame."""
    msg = PointCloud2()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.height      = 1
    msg.width       = len(points_xyzi)
    msg.fields      = _PC2_FIELDS
    msg.is_bigendian = False
    msg.point_step  = _PT_STEP
    msg.row_step    = _PT_STEP * len(points_xyzi)
    msg.is_dense    = True
    buf = bytearray(msg.row_step)
    for i, p in enumerate(points_xyzi):
        struct.pack_into('ffff', buf, i * _PT_STEP,
                         p[0], p[1], p[2],
                         p[3] if len(p) > 3 else 0.0)
    msg.data = bytes(buf)
    return msg


def _quat_to_yaw(w, x, y, z):
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


# ---------------------------------------------------------------------------
# Simulation node
# ---------------------------------------------------------------------------
class MavsSimNode(Node):

    def __init__(self):
        super().__init__('mavs_sim_node')

        # ---- parameters ----
        self.declare_parameter('scene_file',    '')
        self.declare_parameter('vehicle_file',  '')
        self.declare_parameter('lidar_model',   'OS2')
        self.declare_parameter('init_x',        -50.0)
        self.declare_parameter('init_y',        0.0)
        self.declare_parameter('init_z',        0.0)
        self.declare_parameter('init_heading',  0.0)
        self.declare_parameter('dt',            0.01)
        self.declare_parameter('rain_rate',    0.0)
        self.declare_parameter('lidar_rate',    10.0)
        self.declare_parameter('odom_rate',     100.0)
        self.declare_parameter('lidar_height',  1.5)
        self.declare_parameter('desired_speed', 0.0)

        scene_file   = self.get_parameter('scene_file').value
        vehicle_file = self.get_parameter('vehicle_file').value
        lidar_model  = self.get_parameter('lidar_model').value
        init_x       = self.get_parameter('init_x').value
        init_y       = self.get_parameter('init_y').value
        init_z       = self.get_parameter('init_z').value
        init_heading = self.get_parameter('init_heading').value
        self._dt     = self.get_parameter('dt').value
        self.rain_rate   = self.get_parameter('rain_rate').value
        lidar_rate   = self.get_parameter('lidar_rate').value
        odom_rate    = self.get_parameter('odom_rate').value
        lidar_h      = self.get_parameter('lidar_height').value
        self._desired_speed = self.get_parameter('desired_speed').value

        # cavs proving ground origin
        self._lat0   = 33.475626 
        self._lon0   = -88.791577
        self._alt0   = 102.0
        self._gps_frame = 'gps_link'
        
        # ---- MAVS scene & environment ----
        self._scene = mavs.MavsEmbreeScene()
        if scene_file:
            self._scene.Load(scene_file)
            self.get_logger().info(f'Loaded scene: {scene_file}')
        else:
            self.get_logger().warn('No scene_file — using flat terrain')
            tc = mavs.MavsTerrainCreator()
            self._scene.scene = tc.CreateMavsScenePointer(
                -200.0, -200.0, 200.0, 200.0, 0.1)

        self._env = mavs.MavsEnvironment()
        self._env.SetScene(self._scene)
        self._env.SetRainRate(self.rain_rate)

        # ---- MAVS vehicle ----
        self._vehicle = mavs.MavsRp3d()
        if vehicle_file:
            self._vehicle.Load(vehicle_file)
            self.get_logger().info(f'Loaded vehicle: {vehicle_file}')
        else:
            default_veh = os.path.join(
                os.path.dirname(mavs.__file__),
                'data', 'vehicles', 'rp3d_vehicles',
                'forester_2017_rp3d_tires.json')
            self.get_logger().warn(f'No vehicle_file — using {default_veh}')
            self._vehicle.Load(default_veh)

        self._vehicle.SetInitialPosition(init_x, init_y, init_z)
        self._vehicle.SetInitialHeading(init_heading)

        # ---- MAVS lidar ----
        self._lidar = mavs.MavsLidar(lidar_model)
        self._lidar.SetOffset(
            [0.0, 0.0, lidar_h],
            [1.0, 0.0, 0.0, 0.0])
        self.get_logger().info(f'Lidar model: {lidar_model}')

        # ---- MAVS camera ----
        self._debug_camera = mavs.MavsCamera()
        self._debug_camera.Initialize(640, 360, 0.0035*(640.0/360.0), 0.0035, 0.0035)
        self._debug_camera.RenderShadows(True)
        self._debug_camera.SetOffset(
            [-8.0, 0.0, 2.0],
            [1.0, 0.0, 0.0, 0.0]
        )
        
        # ---- MAVS GPS ----
        self._rtk = mavs.MavsRtk()
        self._rtk.SetOffset([0.0, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
        
        # ---- control state (written by cmd_vel callback) ----
        self._throttle = 0.0
        self._steering = 0.0
        self._braking  = 0.0
        self._ctrl_lock = threading.Lock()
        self._physics_ready = False

        # ---- ROS interface ----
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_tf      = tf2_ros.StaticTransformBroadcaster(self)
        self._broadcast_static_tfs()

        self._odom_pub  = self.create_publisher(Odometry,          'nature/odometry', 10)
        self._lidar_pub = self.create_publisher(PointCloud2,        'nature/points',   10)
        self._veh_pub   = self.create_publisher(Float64MultiArray,  'nature/veh',      10)
        self._img_pub  = self.create_publisher(Image, '/camera/image_raw', 10)
        self._fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        self.create_subscription(Twist, 'nature/cmd_vel',
                                 self._cmd_vel_cb, 10)

        #self._lidar_every = max(1, int(round(
        #    self.get_parameter('odom_rate').value /
        #    self.get_parameter('lidar_rate').value)))
        self._lidar_every = max(1, int(round(odom_rate / lidar_rate)))

        # Single thread for ALL mavs calls — lidar uses C++ threads internally
        # and must be called from the same thread as the physics update
        self._mavs_thread = threading.Thread(
            target=self._mavs_loop, daemon=True)
        self._mavs_thread.start()

        self.get_logger().info('MavsSimNode ready.')

    # ------------------------------------------------------------------
    def _broadcast_static_tfs(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id  = 'odom'
        t.transform.rotation.w = 1.0
        self._static_tf.sendTransform(t)

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        """
        nature/cmd_vel convention:
          linear.x  = throttle  (0..1)
          linear.y  = braking   (0..1)
          angular.z = steering  (rad, + = left)
        """
        with self._ctrl_lock:
            self._throttle = float(msg.linear.x)
            self._braking  = float(msg.linear.y)
            self._steering = float(msg.angular.z)

    # ------------------------------------------------------------------
    def _mavs_loop(self):
        """Single thread for ALL MAVS calls.
        The MAVS C++ lidar spins up internal OpenMP threads and must be
        called from the same thread as the physics update to avoid segfaults.
        """
        odom_rate = self.get_parameter('odom_rate').value
        odom_every  = max(1, int(round(1.0 / (odom_rate * self._dt))))
        step = 0

        while rclpy.ok():
            t0 = time.monotonic()

            with self._ctrl_lock:
                throttle = self._throttle
                braking  = self._braking
                steering = self._steering

            if self._desired_speed != 0.0:
                spd = self._vehicle.GetSpeed()
                err = self._desired_speed * throttle - spd
                throttle = max(0.0, min(1.0, err * 0.3))
                braking  = 0.0 if throttle > 0 else 0.5

            self._vehicle.Update(self._env, throttle, steering, braking,
                                 self._dt)
            self._env.AdvanceTime(self._dt)
            self._physics_ready = True

            if step % odom_every == 0:
                self._publish_odometry()

            if step % self._lidar_every == 0:
                self._publish_lidar()
                self._publish_camera()
                self._publish_rtk()

            step += 1
            elapsed = time.monotonic() - t0
            sleep_t = self._dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _publish_rtk(self):

        pos_in = self._vehicle.GetPosition()
        ori_in = self._vehicle.GetOrientation()
        
        self._rtk.SetPose(pos_in, ori_in)
        self._rtk.Update(self._env, 0.1)

        pos = self._rtk.GetPosition()    # ENU with noise
        #ori = self._rtk.GetOrientation()

        stamp = self.get_clock().now().to_msg()
        cov = 0.01  # 10 cm std-dev → 0.01 m² variance

        # --- NavSatFix ---
        lat, lon, alt = enu_to_latlon(pos[0], pos[1], pos[2], self._lat0, self._lon0, self._alt0)

        fix = NavSatFix()
        fix.header.stamp    = stamp
        fix.header.frame_id = self._gps_frame
        fix.status.status   = NavSatStatus.STATUS_FIX
        fix.status.service  = NavSatStatus.SERVICE_GPS
        fix.latitude        = lat
        fix.longitude       = lon
        fix.altitude        = alt
        fix.position_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov * 4,  # altitude typically worse
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(fix)
        
    # ------------------------------------------------------------------
    def _publish_odometry(self):
        now = self.get_clock().now().to_msg()

        pos = self._vehicle.GetPosition()
        vel = self._vehicle.GetVelocity()
        ori = self._vehicle.GetOrientation()   # w, x, y, z
        spd = self._vehicle.GetSpeed()

        # --- TF odom -> base_link ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_link'
        tf_msg.transform.translation.x = pos[0]
        tf_msg.transform.translation.y = pos[1]
        tf_msg.transform.translation.z = pos[2]
        tf_msg.transform.rotation.w = ori[0]
        tf_msg.transform.rotation.x = ori[1]
        tf_msg.transform.rotation.y = ori[2]
        tf_msg.transform.rotation.z = ori[3]
        self._tf_broadcaster.sendTransform(tf_msg)

        # --- nature/odometry ---
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.w = ori[0]
        odom.pose.pose.orientation.x = ori[1]
        odom.pose.pose.orientation.y = ori[2]
        odom.pose.pose.orientation.z = ori[3]
        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[1]
        odom.twist.twist.linear.z = vel[2]
        self._odom_pub.publish(odom)

        # --- nature/veh  [0, x, y, vx, vy, 0, 0, 0, 0] ---
        hdg = self._vehicle.GetHeading()
        veh_msg = Float64MultiArray()
        veh_msg.layout.dim.append(MultiArrayDimension(
            label='x', size=9, stride=1))
        veh_msg.data = [
            0.0,           # index 0 (unused)
            float(pos[0]), # x
            float(pos[1]), # y
            float(vel[0]), # vx
            float(vel[1]), # vy
            float(hdg),    # heading (rad)
            float(spd),    # speed
            0.0,
            0.0,
        ]
        self._veh_pub.publish(veh_msg)

    # ------------------------------------------------------------------
    def _publish_camera(self):
        pos = self._vehicle.GetPosition()
        ori = self._vehicle.GetOrientation()
        self._debug_camera.SetPose(list(pos), list(ori))
        self._debug_camera.Update(self._env, 0.1)
        self._debug_camera.AddLidarPointsToImage(self._lidar)
        stamp = self.get_clock().now().to_msg()

        #import numpy as np
        arr = self._debug_camera.GetNumpyArray()  # returns H x W x 3 uint8 numpy array
        if arr is None:
            return
        h, w = arr.shape[0], arr.shape[1]
        raw = arr.tobytes()

        img = Image()
        img.header.stamp    = stamp
        img.header.frame_id = 'camera'
        img.height    = h
        img.width     = w
        img.encoding  = 'rgb8'
        img.step      = w * 3
        img.data      = raw
        self._img_pub.publish(img)

        
    def _publish_lidar(self):
        """Get lidar scan and publish as PointCloud2 in world (odom) frame."""
        pos = self._vehicle.GetPosition()
        ori = self._vehicle.GetOrientation()

        self._lidar.SetPose(list(pos), list(ori))
        self._lidar.Update(self._env, 0.1)
        
        raw = self._lidar.GetPoints() # these are in world frame

        if not raw:
            return

        #points = [[p[0], p[1], p[2], p[3]] for p in raw]
        points = [[p[0], p[1], p[2]] for p in raw]
        stamp = self.get_clock().now().to_msg()
        pc2 = _make_pc2(points, 'odom', stamp)
        self._lidar_pub.publish(pc2)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
import os

def main(args=None):
    rclpy.init(args=args)
    node = MavsSimNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
