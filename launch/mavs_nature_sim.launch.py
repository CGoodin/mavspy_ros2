"""
mavs_nature_sim.launch.py
--------------------------
Single launch file for the full MAVS + nature-stack simulation.

Usage
-----
  ros2 launch mavspy_ros2 mavs_nature_sim.launch.py

  ros2 launch mavspy_ros2 mavs_nature_sim.launch.py \
      scene_file:=/path/to/scene.json \
      waypoints_file:=/path/to/waypoints.yaml \
      robot_description_file:=/path/to/robot.urdf \
      run_viz:=true
"""

import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ---------------------------------------------------------------------------
# Resolve paths at import time (before any LaunchConfiguration exists)
# ---------------------------------------------------------------------------
def _find_mavs_data():
    env = os.environ.get('MAVS_DATA', '')
    if env and os.path.isdir(env):
        return env
    try:
        r = subprocess.run(
            [sys.executable, '-c',
             'import mavspy, os; '
             'print(os.path.dirname(mavspy.__file__)+"/data", end="")'],
            capture_output=True, text=True, timeout=5)
        if r.returncode == 0 and r.stdout.strip():
            return r.stdout.strip()
    except Exception:
        pass
    return ''


def _find_nature_launch():
    try:
        pkg = get_package_share_directory('nature')
        p = os.path.join(pkg, 'launch', 'base.launch.py')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return ''


def _find_viz():
    # 1. Find via installed Python module (most reliable after colcon build)
    try:
        import mavspy_ros2
        p = os.path.join(os.path.dirname(mavspy_ros2.__file__), 'mavs_viz_x11.py')
        if os.path.exists(p):
            return os.path.abspath(p)
    except ImportError:
        pass

    # 2. Find via ament share directory
    try:
        pkg = get_package_share_directory('mavspy_ros2')
        # installed scripts land in lib/mavspy_ros2/
        p = os.path.join(os.path.dirname(pkg), '..', 'lib',
                         'mavspy_ros2', 'mavs_viz_x11.py')
        if os.path.exists(p):
            return os.path.abspath(p)
    except Exception:
        pass

    # 3. Search relative to this launch file
    this_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(this_dir, '..', 'mavspy_ros2', 'mavs_viz_x11.py'),
        os.path.join(this_dir, '..', '..', 'mavspy_ros2', 'mavs_viz_x11.py'),
        'mavs_viz_x11.py',
    ]
    for p in candidates:
        if os.path.exists(p):
            return os.path.abspath(p)

    # 4. Walk the colcon workspace
    ws = os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0]
    if ws:
        for root, _, files in os.walk(os.path.dirname(ws)):
            if 'mavs_viz_x11.py' in files:
                return os.path.join(root, 'mavs_viz_x11.py')
    return ''


MAVS_DATA     = _find_mavs_data()
NATURE_LAUNCH = _find_nature_launch()
VIZ_PATH      = _find_viz()


# ---------------------------------------------------------------------------
def _write_mavs_config(context):
    data_path = MAVS_DATA
    if data_path:
        config_path = os.path.join(os.path.expanduser('~'), 'mavs_config.txt')
        try:
            with open(config_path, 'w') as f:
                f.write(data_path)
            print(f'[mavs_nature_sim] mavs_config.txt -> {data_path}')
        except Exception as e:
            print(f'[mavs_nature_sim] warning: could not write config: {e}')
    return []


def _launch_nature(context):
    wf = context.launch_configurations.get('waypoints_file', '')
    uf = context.launch_configurations.get('robot_description_file', '')

    if not NATURE_LAUNCH:
        print('[mavs_nature_sim] nature package not found — start it manually')
        return []
    if not wf:
        print('[mavs_nature_sim] No waypoints_file — skipping nature launch')
        return []
    if not uf or not os.path.exists(uf):
        print(f'[mavs_nature_sim] robot_description_file not found: {uf}')
        return []
    try:
        with open(uf) as f:
            robot_desc = f.read()
    except Exception as e:
        print(f'[mavs_nature_sim] could not read URDF: {e}')
        return []

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(NATURE_LAUNCH),
        launch_arguments={
            'waypoints_file':    wf,
            'robot_description': robot_desc,
            'grid_llx': '-150.0',
            'grid_lly': '-175.0',
            'grid_height': '500.0',
            'grid_width': '500.0',
            'grid_res': '1.0',
            'use_registered': 'True'
        }.items(),
    )]


def _launch_viz(context):
    if context.launch_configurations.get('run_viz', 'true').lower() != 'true':
        return []
    viz = VIZ_PATH
    if not viz:
        print('[mavs_nature_sim] mavs_viz_x11.py not found — run it manually')
        return []
    from launch.actions import ExecuteProcess
    # Pass DISPLAY through so X11 works inside Apptainer
    display = os.environ.get('DISPLAY', ':0')
    return [ExecuteProcess(
        cmd=[sys.executable, viz],
        output='screen',
        additional_env={'DISPLAY': display},
    )]


# ---------------------------------------------------------------------------
def generate_launch_description():

    # Default paths
    default_scene = os.path.join(MAVS_DATA, 'scenes', 'cube_scene.json') \
        if MAVS_DATA else ''

    try:
        nature_share    = get_package_share_directory('nature')
        default_wpts    = os.path.join(nature_share, 'config', 'waypoints.yaml')
        default_urdf    = os.path.join(nature_share, 'config', 'example_bot.urdf')
    except Exception:
        default_wpts = ''
        default_urdf = ''

    print(f'[mavs_nature_sim] MAVS data:      {MAVS_DATA or "NOT FOUND"}')
    print(f'[mavs_nature_sim] nature launch:  {NATURE_LAUNCH or "NOT FOUND"}')
    print(f'[mavs_nature_sim] viz:            {VIZ_PATH or "NOT FOUND"}')

    args = [
        DeclareLaunchArgument('scene_file',
            default_value=default_scene,
            description='MAVS scene JSON'),
        DeclareLaunchArgument('vehicle_file',
            default_value='',
            description='MAVS vehicle JSON (blank = default forester)'),
        DeclareLaunchArgument('lidar_model',
            default_value='VLP-16',
            description='MAVS lidar model'),
        DeclareLaunchArgument('waypoints_file',
            default_value=default_wpts,
            description='nature waypoints YAML'),
        DeclareLaunchArgument('robot_description_file',
            default_value=default_urdf,
            description='Robot URDF for nature stack'),
        DeclareLaunchArgument('run_viz',
            default_value='true',
            description='Launch X11 matplotlib visualizer'),
        DeclareLaunchArgument('init_x',
            default_value='0.0',
            description='Initial vehicle ENU X position (m)'),
        DeclareLaunchArgument('init_y',
            default_value='0.0',
            description='Initial vehicle ENU Y position (m)'),
        DeclareLaunchArgument('init_z',
            default_value='0.0',
            description='Initial vehicle ENU Z position (m)'),
        DeclareLaunchArgument('init_heading',
            default_value='0.0',
            description='Initial vehicle heading (rad)'),
    ]

    # Set env vars using plain strings (not LaunchConfiguration — avoids
    # the "configuration does not exist" error when used before declaration)
    env_actions = []
    if MAVS_DATA:
        env_actions.append(SetEnvironmentVariable('MAVS_DATA', MAVS_DATA))

    def _launch_mavs_sim(context):
        scene   = context.launch_configurations.get('scene_file', '')
        vehicle = context.launch_configurations.get('vehicle_file', '')
        lidar   = context.launch_configurations.get('lidar_model', 'VLP-16')
        return [Node(
            package='mavspy_ros2',
            executable='mavs_sim_node',
            name='mavs_sim_node',
            output='screen',
            parameters=[{
                'scene_file':   scene,
                'vehicle_file': vehicle,
                'lidar_model':  lidar,
                'lidar_rate':   10.0,
                'odom_rate':    50.0,
                'use_sim_time': False,
                'init_x':       float(context.launch_configurations.get('init_x', '0.0')),
                'init_y':       float(context.launch_configurations.get('init_y', '0.0')),
                'init_z':       float(context.launch_configurations.get('init_z', '0.0')),
                'init_heading': float(context.launch_configurations.get('init_heading', '0.0')),
            }],
        )]

    return LaunchDescription(
        env_actions +
        args +
        [
            OpaqueFunction(function=_write_mavs_config),
            OpaqueFunction(function=_launch_mavs_sim),
            OpaqueFunction(function=_launch_nature),
            OpaqueFunction(function=_launch_viz),
        ]
    )
