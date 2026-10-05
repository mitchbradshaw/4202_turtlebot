#!/usr/bin/env python3
"""Generate, validate and preview the metr4202_test_worlds Gazebo worlds.

Everything under worlds/, launch/<world>.launch.py, docs/*.png and
resource/media/materials is produced by this script, so the mazes are defined
in exactly one place (the WORLDS table below).

    python3 tools/generate_worlds.py              # worlds + launch files + validation report
    python3 tools/generate_worlds.py --previews   # also docs/<world>.png   (needs matplotlib)
    python3 tools/generate_worlds.py --textures   # also marker textures    (needs opencv-contrib)

You do NOT need to run this to use the package; the generated files are
committed. Run it only if you edit a maze.

Conventions (same as metr4202_aruco_explore):
  * Each ASCII character is one 0.5 m x 0.5 m cell; '#' is a 1 m tall wall.
    A 21 x 21 map is the reference 10 m arena: outer wall centre-lines at
    +/-5 m, drivable interior from -4.75 m to +4.75 m.
  * Row 0 is the +y edge, column 0 is the -x edge (map is drawn x right, y up).
  * A marker is the reference "ArUco column": 0.12 m square post with a
    100 mm DICT_6X6_1000 tag on its local -x face. With model yaw psi the
    tag faces (-cos psi, -sin psi).
"""

import argparse
import json
import math
import os
import sys

import numpy as np

CELL = 0.5
WALL_H = 1.0
COLUMN_W = 0.12
MARKER_SIZE = 0.10

# TurtleBot3 Waffle Pi numbers used by the validator.
ROBOT_RADIUS = 0.22          # nav2 robot_radius for waffle_pi
CAMERA_Z = 0.11              # camera optical centre height above ground
CAMERA_HFOV = 1.085595       # rad, 640 px wide
CAMERA_VFOV = 2 * math.atan(math.tan(CAMERA_HFOV / 2) * 480 / 640)
FOCAL_PX = 320 / math.tan(CAMERA_HFOV / 2)
LIDAR_RANGE = 3.5

# "Readable" = the tag's short side spans at least this many pixels and is not
# seen more obliquely than this. Conservative numbers for cv2.aruco at 640x480.
MIN_TAG_PX = 20.0
MAX_INCIDENCE = math.radians(60)

PI = math.pi
FACE_NEG_X, FACE_POS_X = 0.0, PI          # yaw needed for the tag to face -x / +x
FACE_NEG_Y, FACE_POS_Y = PI / 2, -PI / 2  # ... to face -y / +y


def marker(mid, x, y, yaw, note, z=0.24, height=0.3, roll=0.0, pitch=0.0, base_z=0.0, label_shift=0.0):
    """z = tag centre height up the post (reference model: 0.24), base_z = height of the post's foot."""
    return dict(id=mid, x=x, y=y, yaw=yaw, z=z, height=height, roll=roll, pitch=pitch, base_z=base_z,
                label_shift=label_shift, note=note)


def box(cx, cy, sx, sy, yaw=0.0, note=''):
    return dict(kind='box', cx=cx, cy=cy, sx=sx, sy=sy, yaw=yaw, note=note)


def cyl(cx, cy, r, note=''):
    return dict(kind='cyl', cx=cx, cy=cy, r=r, note=note)


WORLDS = {}

# --------------------------------------------------------------------------
# 1. HORSESHOE
# --------------------------------------------------------------------------
WORLDS['horseshoe'] = dict(
    title='Horseshoe',
    purpose='U-shaped arena. The far tip is ~7 m from the start in a straight line but ~18 m away '
            'by road; a barred window lets the lidar (not the robot) see from one arm into the other.',
    ascii=[
        '#####################',
        '#.......#####.......#',
        '#.......###.........#',
        '#.......###.........#',
        '#.......#####.......#',
        '#####...#####.......#',
        '#.......#####...#####',
        '#...................#',
        '#...................#',
        '#...#########.......#',
        '#.......#########...#',
        '#.......#####.......#',
        '#.......#####.......#',
        '#.......#####.......#',
        '#...................#',
        '#...................#',
        '#........###........#',
        '#........###........#',
        '#...................#',
        '#...................#',
        '#####################',
    ],
    extras=[
        box(0.0, 1.675, 0.1, 0.1, 0.0, 'window bar'),
        box(0.0, 1.400, 0.1, 0.1, 0.0, 'window bar'),
        box(0.0, 1.125, 0.1, 0.1, 0.0, 'window bar'),
        box(0.0, 0.850, 0.1, 0.1, 0.0, 'window bar'),
    ],
    spawn=(-3.0, 3.5, -PI / 2),
    markers=[
        marker(0, -3.0, 4.69, FACE_NEG_Y, 'directly BEHIND the spawn pose (robot starts facing away)'),
        marker(2, -4.0, -4.69, FACE_POS_Y, 'bottom of the left arm, seen head-on after the 2nd baffle'),
        marker(42, 0.31, 3.75, FACE_POS_X, 'back of the 1 m pocket cut into the divider (right arm tip)'),
        marker(7, 4.0, 4.69, FACE_NEG_Y, 'far tip of the right arm = end of the horseshoe'),
    ],
)

# --------------------------------------------------------------------------
# 2. MARKER GAUNTLET
# --------------------------------------------------------------------------
WORLDS['marker_gauntlet'] = dict(
    title='Marker gauntlet',
    purpose='Easy to drive, hard to read: eight tags that each defeat a different '
            'lazy assumption about where a tag will be and how it will look.',
    ascii=[
        '#####################',
        '#...................#',
        '#...................#',
        '####..#..#.#######..#',
        '####..#..#.#######..#',
        '####..#..#########..#',
        '#.....#......#......#',
        '#.....#......#......#',
        '#.....#......#......#',
        '#.....#......#......#',
        '#.....#......#......#',
        '#.....#......#......#',
        '#.....#......#......#',
        '#.....#......#......#',
        '#..########..#..#####',
        '#..########..#..#####',
        '#...................#',
        '#...................#',
        '#...................#',
        '#...................#',
        '#####################',
    ],
    extras=[
        cyl(-0.75, -1.15, 0.10, 'pillar: centre 0.48 m in front of the face of tag 13'),
        box(-0.225, 3.25, 0.05, 1.0, 0.0, 'slot liner: narrows the slot to 0.4 m'),
        box(0.225, 3.25, 0.05, 1.0, 0.0, 'slot liner'),
    ],
    spawn=(-4.25, 4.25, 0.0),
    markers=[
        marker(2, -4.69, -0.75, FACE_POS_X, 'HIGH: 0.9 m up a 1 m post on the side wall of a 2.5 m wide room',
               z=0.90, height=1.0),
        marker(42, 0.0, 2.81, FACE_POS_Y, 'RECESSED: back of a 0.4 m wide, 1 m deep slot the robot cannot enter'),
        marker(7, -4.0, 2.19, FACE_NEG_Y, 'BEHIND YOU: beside the north door of room A, facing into the room'),
        marker(13, -0.75, -1.69, FACE_POS_Y, 'OCCLUDED: pillar in front, hidden from a band straight ahead of it'),
        marker(21, 1.19, 0.90, FACE_NEG_X, 'PAIR (left): two different IDs 14 cm apart', label_shift=0.3),
        marker(22, 1.19, 0.76, FACE_NEG_X, 'PAIR (right)', label_shift=-0.3),
        marker(64, 3.25, 0.25, FACE_NEG_Y, 'TILTED: 0.5 m post leaning back 45 deg, tag faces half-up and is '
               'rotated 20 deg in its own plane', z=0.45, height=0.5,
               roll=math.radians(20), pitch=math.radians(45), base_z=0.05),
        marker(0, -2.5, -4.69, FACE_POS_Y, 'GRAZING: flat on the side wall of a 2 m wide corridor'),
    ],
)

# --------------------------------------------------------------------------
# 3. SLAM STRESS
# --------------------------------------------------------------------------
WORLDS['slam_stress'] = dict(
    title='SLAM stress',
    purpose='Geometry that is awkward for scan matching and costmaps: a featureless corridor longer '
            'than the lidar range, repeated bays, pillars, diagonal and paper-thin walls, a 0.75 m '
            'doorway, and a single loop that can only be closed through the featureless corridor.',
    ascii=[
        '#####################',
        '#...#...#...#...#...#',
        '#...#...#...#...#...#',
        '#...................#',
        '#...................#',
        '#######..###..#######',
        '#.........#.........#',
        '#.........#.........#',
        '#.........#.....#...#',
        '#.........#.....#####',
        '#.........#.........#',
        '#.........#.........#',
        '#.........#.........#',
        '#.........#.........#',
        '#.........#.........#',
        '#.........#.........#',
        '#.........#.........#',
        '#..###############..#',
        '#...................#',
        '#...................#',
        '#####################',
    ],
    extras=[
        cyl(-3.6, 1.1, 0.08, 'pillar'),
        cyl(-2.1, 0.9, 0.08, 'pillar'),
        cyl(-3.0, -0.4, 0.08, 'pillar'),
        cyl(-1.4, -0.2, 0.08, 'pillar'),
        cyl(-3.7, -1.9, 0.08, 'pillar'),
        cyl(-1.3, -2.2, 0.08, 'pillar'),
        box(1.6, -0.1, 1.4, 0.15, -PI / 4, 'diagonal wall'),
        box(3.1, -1.6, 2.0, 0.15, PI / 4, 'diagonal wall'),
        box(1.5, -2.625, 0.05, 1.25, 0.0, '5 cm thin partition'),
        box(3.0, 2.125, 0.5, 0.25, 0.0, 'door stub: narrows the closet doorway to 0.75 m'),
    ],
    spawn=(-4.0, 3.25, 0.0),
    markers=[
        marker(0, 2.0, 4.69, FACE_NEG_Y, 'back wall of the 4th of five identical bays'),
        marker(2, 4.0, 0.81, FACE_POS_Y, 'inside the closet, behind the 0.75 m doorway'),
        marker(42, 4.69, -4.25, FACE_NEG_X, 'east end of the 9.5 m featureless corridor'),
        marker(7, -2.3, -1.3, FACE_NEG_X, 'free-standing post among the pillars, faces west'),
    ],
)


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------
def grid_dims(w):
    return len(w['ascii']), len(w['ascii'][0])


def cell_centre(w, r, c):
    rows, cols = grid_dims(w)
    return (-(cols - 1) * CELL / 2 + c * CELL, (rows - 1) * CELL / 2 - r * CELL)


def wall_rectangles(w, which='all'):
    """Greedy-merge '#' cells into axis-aligned boxes: list of (cx, cy, sx, sy).

    which = 'outer' (boundary ring), 'inner' (everything else) or 'all' (outer then inner).
    """
    if which == 'all':
        return wall_rectangles(w, 'outer') + wall_rectangles(w, 'inner')
    rows, cols = grid_dims(w)

    def wanted(r, c):
        edge = r in (0, rows - 1) or c in (0, cols - 1)
        return w['ascii'][r][c] == '#' and edge == (which == 'outer')
    g = [[wanted(r, c) for c in range(cols)] for r in range(rows)]
    used = [[False] * cols for _ in range(rows)]
    rects = []
    for r in range(rows):
        for c in range(cols):
            if not g[r][c] or used[r][c]:
                continue
            c2 = c
            while c2 + 1 < cols and g[r][c2 + 1] and not used[r][c2 + 1]:
                c2 += 1
            r2 = r
            while r2 + 1 < rows and all(g[r2 + 1][k] and not used[r2 + 1][k] for k in range(c, c2 + 1)):
                r2 += 1
            for rr in range(r, r2 + 1):
                for cc in range(c, c2 + 1):
                    used[rr][cc] = True
            x0, y0 = cell_centre(w, r, c)
            x1, y1 = cell_centre(w, r2, c2)
            rects.append(((x0 + x1) / 2, (y0 + y1) / 2, (c2 - c + 1) * CELL, (r2 - r + 1) * CELL))
    return rects


def marker_frame(m):
    """World-frame tag centre, outward normal and in-plane horizontal axis."""
    cr, sr = math.cos(m['roll']), math.sin(m['roll'])
    cp, sp = math.cos(m['pitch']), math.sin(m['pitch'])
    cy, sy = math.cos(m['yaw']), math.sin(m['yaw'])
    rot = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ \
        np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ \
        np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    centre = np.array([m['x'], m['y'], m['base_z']]) + rot @ np.array([-COLUMN_W / 2 - 0.0005, 0.0, m['z']])
    normal = rot @ np.array([-1.0, 0.0, 0.0])
    side = rot @ np.array([0.0, 1.0, 0.0])
    up = rot @ np.array([0.0, 0.0, 1.0])
    return centre, normal, side, up


# --------------------------------------------------------------------------
# SDF emission
# --------------------------------------------------------------------------
MATERIAL = """<material>
            <script>
              <name>Gazebo/%s</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>"""


def _geom(tag, name, pose, geometry, colour):
    extra = MATERIAL % colour if tag == 'visual' else '<max_contacts>10</max_contacts>'
    return f"""        <{tag} name='{name}'>
          <pose>{pose}</pose>
          <geometry>
            {geometry}
          </geometry>
          {extra}
        </{tag}>
"""


def maze_model(w):
    parts = []
    # Reference colours: boundary walls black, interior walls grey.
    shapes = [(f'{which}_wall_{i}', f'{cx:.4f} {cy:.4f} {WALL_H / 2} 0 0 0',
               f'<box><size>{sx:.4f} {sy:.4f} {WALL_H}</size></box>', colour)
              for which, colour in (('outer', 'Black'), ('inner', 'Grey'))
              for i, (cx, cy, sx, sy) in enumerate(wall_rectangles(w, which))]
    for i, e in enumerate(w['extras']):
        if e['kind'] == 'box':
            shapes.append((f'extra_{i}', f"{e['cx']:.4f} {e['cy']:.4f} {WALL_H / 2} 0 0 {e['yaw']:.5f}",
                           f"<box><size>{e['sx']:.4f} {e['sy']:.4f} {WALL_H}</size></box>", 'Grey'))
        else:
            shapes.append((f'extra_{i}', f"{e['cx']:.4f} {e['cy']:.4f} {WALL_H / 2} 0 0 0",
                           f"<cylinder><radius>{e['r']:.4f}</radius><length>{WALL_H}</length></cylinder>",
                           'Grey'))
    for name, pose, geometry, colour in shapes:
        parts.append(_geom('collision', name + '_collision', pose, geometry, colour))
        parts.append(_geom('visual', name + '_visual', pose, geometry, colour))
    return f"""    <model name='maze'>
      <static>1</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
{''.join(parts)}      </link>
    </model>
"""


def marker_model(m):
    """The reference ArUco column, flattened to one static link.

    Same post, same 100 mm tag plate in the same place and orientation (rpy 0 pi/2 0 is the
    rotation the reference uses), but written with plain poses instead of the reference's
    second link + fixed joint + SDF <frame>, so nothing depends on frame-semantics support.
    """
    i, h = m['id'], m['height']
    return f"""    <model name='aruco_column_6x6_100mm_{i}'>
      <static>1</static>
      <pose>{m['x']:.4f} {m['y']:.4f} {m['base_z']:.4f} {m['roll']:.5f} {m['pitch']:.5f} {m['yaw']:.5f}</pose>
      <link name='link'>
        <collision name='collision'>
          <pose>0 0 {h / 2:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{COLUMN_W} {COLUMN_W} {h}</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <pose>0 0 {h / 2:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{COLUMN_W} {COLUMN_W} {h}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <visual name='aruco_6x6_100mm_{i}'>
          <pose>{-COLUMN_W / 2} 0 {m['z']} 0 1.5708 0</pose>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>{MARKER_SIZE} {MARKER_SIZE} 0.001</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts</uri>
              <uri>file://media/materials/textures</uri>
              <name>tw_aruco_6x6_100mm_{i}</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""


WORLD_HEAD = """<?xml version='1.0'?>
<!-- GENERATED by tools/generate_worlds.py - edit the WORLDS table there, not this file. -->
<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0 0 20 0 1.57 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
"""


def world_sdf(w, with_aruco=True):
    out = [WORLD_HEAD, maze_model(w)]
    if with_aruco:
        out += [marker_model(m) for m in w['markers']]
    out.append('  </world>\n</sdf>\n')
    return ''.join(out)


# --------------------------------------------------------------------------
# Launch file template (one self-contained launch file per world)
# --------------------------------------------------------------------------
LAUNCH_TEMPLATE = '''#!/usr/bin/env python3
# GENERATED by tools/generate_worlds.py - edit the template there, not this file.
"""Launch the '@NAME@' test world with a TurtleBot3 (default: waffle_pi).

    ros2 launch metr4202_test_worlds @NAME@.launch.py
    ros2 launch metr4202_test_worlds @NAME@.launch.py aruco:=false gui:=false
    ros2 launch metr4202_test_worlds @NAME@.launch.py odometry:=encoder odom_error:=0.03
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

WORLD = '@NAME@'
DEFAULT_X, DEFAULT_Y, DEFAULT_YAW = '@X@', '@Y@', '@YAW@'


def spawn_robot(context, sdf_path, tb3_model):
    """Spawn the TurtleBot3; with odometry:=encoder, from a copy whose /odom comes from the wheels.

    The stock model leaves gazebo_ros_diff_drive's <odometry_source> at its default (1 = WORLD), so
    /odom is the exact Gazebo pose and never drifts. Source 0 (ENCODER) integrates wheel rotation
    instead, starts at zero and accumulates error like a real robot.
    """
    actions = []
    odometry = LaunchConfiguration('odometry').perform(context).lower()
    odom_error = float(LaunchConfiguration('odom_error').perform(context))
    if odometry not in ('world', 'encoder'):
        actions.append(LogInfo(msg="odometry:=" + odometry + " is not 'world' or 'encoder'; using world"))
    if odometry == 'encoder':
        with open(sdf_path, 'r') as infp:
            sdf = infp.read()

        def miscalibrate(match):
            # The plugin believes the wheels are (1 + odom_error) times their real size and spacing.
            return match.group(1) + repr(float(match.group(2)) * (1.0 + odom_error)) + match.group(3)
        if odom_error:
            sdf = re.sub(r'(<wheel_(?:separation|diameter)>)\\s*([-+.\\deE]+)\\s*(</wheel_(?:separation|diameter)>)',
                         miscalibrate, sdf)
        sdf = re.sub(r'\\s*<odometry_source>.*?</odometry_source>', '', sdf)
        sdf, count = re.subn(r'(<plugin[^>]*libgazebo_ros_diff_drive\\.so[^>]*>)',
                             r'\\1\\n      <odometry_source>0</odometry_source>', sdf)
        if count == 1:
            patched = os.path.join(tempfile.gettempdir(), 'metr4202_test_worlds_%s_encoder_%d.sdf'
                                   % (tb3_model, os.getuid()))
            with open(patched, 'w') as outfp:
                outfp.write(sdf)
            sdf_path = patched
            actions.append(LogInfo(msg='odometry:=encoder (odom_error %g) -> spawning %s' % (odom_error, patched)))
        else:
            actions.append(LogInfo(msg='odometry:=encoder ignored: diff drive plugin not found in ' + sdf_path))
    actions.append(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', tb3_model,
            '-file', sdf_path,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.01',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    ))
    return actions


def generate_launch_description():
    package_dir = get_package_share_directory('metr4202_test_worlds')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Same behaviour as the reference package, except the model defaults to waffle_pi.
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    sdf_path = os.path.join(pkg_tb3_gazebo, 'models', 'turtlebot3_' + tb3_model, 'model.sdf')
    urdf_path = os.path.join(pkg_tb3_gazebo, 'urdf', 'turtlebot3_' + tb3_model + '.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    aruco = LaunchConfiguration('aruco')

    # The ArUco materials/textures live in <share>/resource/media, exactly like
    # metr4202_aruco_explore. Gazebo's own resources must stay on the path too.
    resource_path = os.path.join(package_dir, 'resource')
    if os.environ.get('GAZEBO_RESOURCE_PATH'):
        resource_path = os.environ['GAZEBO_RESOURCE_PATH'] + os.pathsep + resource_path
    else:
        resource_path = '/usr/share/gazebo-11' + os.pathsep + resource_path

    world = PythonExpression([
        "'", os.path.join(package_dir, 'worlds', WORLD), "' + ",
        "('.world' if '", aruco, "'.lower() in ('true', '1', 'yes') else '_no_aruco.world')",
    ])

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
    )
    spawn_turtlebot_cmd = OpaqueFunction(function=spawn_robot, args=[sdf_path, tb3_model])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('aruco', default_value='true',
                              description='false = same maze without the ArUco columns'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = run gzserver only (headless)'),
        DeclareLaunchArgument('x_pose', default_value=DEFAULT_X, description='spawn x [m]'),
        DeclareLaunchArgument('y_pose', default_value=DEFAULT_Y, description='spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value=DEFAULT_YAW, description='spawn heading [rad]'),
        DeclareLaunchArgument('odometry', default_value='world',
                              description='world = stock model (/odom is ground truth); '
                                          'encoder = wheel odometry that starts at zero and drifts'),
        DeclareLaunchArgument('odom_error', default_value='0.0',
                              description='with odometry:=encoder: fractional wheel calibration error, '
                                          'e.g. 0.03 makes /odom over-read distance and under-read turns'),
        LogInfo(msg='GAZEBO_RESOURCE_PATH: ' + resource_path),
        LogInfo(msg='TurtleBot3 model: ' + tb3_model + ' (waffle_pi is the one with a camera)'),
        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=resource_path),
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value=tb3_model),
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
    ])
'''


def launch_file(name, w):
    x, y, yaw = w['spawn']
    return (LAUNCH_TEMPLATE.replace('@NAME@', name).replace('@X@', f'{x:g}')
            .replace('@Y@', f'{y:g}').replace('@YAW@', f'{yaw:.4f}'))


def material_script(ids):
    return '\n'.join(f"""material tw_aruco_6x6_100mm_{i}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture tw_6x6_1000-{i}.png
        filtering none none none
        scale 1.0 1.0
      }}
    }}
  }}
}}
""" for i in ids)


# --------------------------------------------------------------------------
# Validation: can the robot get everywhere, and can every tag be read?
# --------------------------------------------------------------------------
RES = 0.025


class Raster:
    def __init__(self, w):
        rows, cols = grid_dims(w)
        self.x0, self.y0 = -cols * CELL / 2, -rows * CELL / 2
        self.nx, self.ny = int(round(cols * CELL / RES)), int(round(rows * CELL / RES))
        xs = self.x0 + (np.arange(self.nx) + 0.5) * RES
        ys = self.y0 + (np.arange(self.ny) + 0.5) * RES
        self.X, self.Y = np.meshgrid(xs, ys, indexing='ij')
        occ = np.zeros((self.nx, self.ny), bool)
        for cx, cy, sx, sy in wall_rectangles(w):
            occ |= (abs(self.X - cx) <= sx / 2) & (abs(self.Y - cy) <= sy / 2)
        for e in w['extras']:
            if e['kind'] == 'cyl':
                occ |= np.hypot(self.X - e['cx'], self.Y - e['cy']) <= e['r']
            else:
                c, s = math.cos(e['yaw']), math.sin(e['yaw'])
                dx, dy = self.X - e['cx'], self.Y - e['cy']
                occ |= (abs(c * dx + s * dy) <= e['sx'] / 2) & (abs(-s * dx + c * dy) <= e['sy'] / 2)
        self.walls = occ.copy()
        self.columns = {m['id']: (abs(self.X - m['x']) <= COLUMN_W / 2) & (abs(self.Y - m['y']) <= COLUMN_W / 2)
                        for m in w['markers']}
        for col in self.columns.values():
            occ = occ | col
        self.occ = occ

    def sight_blockers(self, marker_id):
        """Everything that can hide a tag: walls, extras and the OTHER tags' columns."""
        occ = self.walls.copy()
        for mid, col in self.columns.items():
            if mid != marker_id:
                occ |= col
        return occ

    def idx(self, x, y):
        return int((x - self.x0) / RES), int((y - self.y0) / RES)

    def inflate(self, occ, radius):
        n = int(math.ceil(radius / RES))
        out = occ.copy()
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy <= (radius / RES) ** 2:
                    out |= np.roll(np.roll(occ, dx, 0), dy, 1)
        return out

    def flood(self, blocked, start):
        seen = np.zeros_like(blocked)
        i, j = self.idx(*start)
        if blocked[i, j]:
            return seen
        seen[i, j] = True
        frontier = seen.copy()
        while frontier.any():
            grown = np.zeros_like(frontier)
            grown[1:, :] |= frontier[:-1, :]
            grown[:-1, :] |= frontier[1:, :]
            grown[:, 1:] |= frontier[:, :-1]
            grown[:, :-1] |= frontier[:, 1:]
            frontier = grown & ~blocked & ~seen
            seen |= frontier
        return seen

    def los(self, occ, ax, ay, bx, by):
        n = max(2, int(math.hypot(bx - ax, by - ay) / (RES * 0.5)))
        t = (np.arange(1, n) / n)
        i = ((ax + (bx - ax) * t - self.x0) / RES).astype(int)
        j = ((ay + (by - ay) * t - self.y0) / RES).astype(int)
        return not occ[i, j].any()


def readable_from(m, ras, blockers, px, py):
    """Could a camera at (px, py, CAMERA_Z), pointed at the tag, decode it?"""
    centre, normal, side, up = marker_frame(m)
    v = np.array([px, py, CAMERA_Z]) - centre
    dist = float(np.linalg.norm(v))
    if dist < 0.3:
        return False
    cos_inc = float(v @ normal) / dist
    if cos_inc < math.cos(MAX_INCIDENCE):
        return False
    # apparent size of the tag's two axes, in pixels
    v_hat = v / dist
    px_side = FOCAL_PX * MARKER_SIZE * float(np.linalg.norm(side - (side @ v_hat) * v_hat)) / dist
    px_up = FOCAL_PX * MARKER_SIZE * float(np.linalg.norm(up - (up @ v_hat) * v_hat)) / dist
    if min(px_side, px_up) < MIN_TAG_PX:
        return False
    # whole tag inside the vertical field of view with the camera level
    for sgn in (-1, 1):
        corner = centre + sgn * up * MARKER_SIZE / 2
        horiz = math.hypot(corner[0] - px, corner[1] - py)
        if abs(math.atan2(corner[2] - CAMERA_Z, horiz)) > CAMERA_VFOV / 2 - math.radians(2):
            return False
    # line of sight to both vertical edges and the centre (walls/pillars/columns are all taller)
    for k in (-1, 0, 1):
        p = centre + k * side * MARKER_SIZE / 2 + normal * 0.03
        if not ras.los(blockers, px, py, p[0], p[1]):
            return False
    return True


def in_spawn_frame(w, p):
    """World point -> frame whose origin/heading is the spawn pose (= slam_toolbox's map frame)."""
    sx, sy, syaw = w['spawn']
    dx, dy = p[0] - sx, p[1] - sy
    c, s = math.cos(syaw), math.sin(syaw)
    return [round(c * dx + s * dy, 3), round(-s * dx + c * dy, 3), round(float(p[2]), 3)]


def ground_truth_md(summary):
    out = ['# Ground truth', '',
           'GENERATED by `tools/generate_worlds.py`. Positions are the centre of the tag face, in metres.', '',
           '* **world** = Gazebo world frame. With the stock TurtleBot3 Gazebo model `/odom` is expected to be',
           '  the true world pose, so `odom`, and therefore the SLAM `map` frame, coincide with this frame.',
           '* **spawn frame** = origin and +x axis at the default spawn pose. This is where `map` sits instead',
           '  if your odometry starts at zero (for example `odometry:=encoder`).',
           '* Only the tag centre and the direction it faces are ground truth. The rotation of the pattern',
           '  within its own plane was not measured, so do not score roll about the normal.',
           '* Not sure which applies? Run `ros2 topic echo /odom --once` right after launch: if it shows the',
           '  spawn pose use the world columns, if it shows zeros use the spawn-frame columns.',
           '* **readable area** = floor area the robot centre can occupy while a level 640x480 Waffle Pi camera',
           f'  pointed at the tag sees it at least {MIN_TAG_PX:.0f} px tall/wide, no more than '
           f'{math.degrees(MAX_INCIDENCE):.0f} deg off its normal and unoccluded.',
           '  It is a geometric estimate of difficulty (smaller = harder), not a Gazebo measurement.', '']
    for name, rep in summary.items():
        sp = rep['spawn']
        out += [f'## {name}', '',
                f"Spawn: x = {sp['x']}, y = {sp['y']}, yaw = {sp['yaw']} rad. "
                f"Drivable floor: {rep['reachable_m2']} m^2.", '',
                '| ID | world x, y, z | spawn-frame x, y, z | faces (world) | readable area | max range | what it tests |',
                '|---:|---|---|---|---:|---:|---|']
        for m in rep['markers']:
            wx, wy, wz = m['tag_centre']
            mx, my, mz = m['tag_centre_in_spawn_frame']
            nx, ny, nz = m['tag_normal']
            out.append(f"| {m['id']} | {wx:+.2f}, {wy:+.2f}, {wz:.2f} | {mx:+.2f}, {my:+.2f}, {mz:.2f} | "
                       f"{nx:+.2f}, {ny:+.2f}, {nz:+.2f} | {m['readable_area_m2']:.2f} m^2 | "
                       f"{m['max_read_range_m']} m | {m['note']} |")
        out.append('')
    return chr(10).join(out)


def validate(name, w):
    ras = Raster(w)
    rows, cols = grid_dims(w)
    problems = []
    if any(len(r) != cols for r in w['ascii']):
        problems.append('ragged ASCII map')
    blocked = ras.inflate(ras.occ, ROBOT_RADIUS)
    reach = ras.flood(blocked, w['spawn'][:2])
    if not reach.any():
        problems.append('spawn pose is in collision')
    free = ~blocked
    stranded = (free & ~reach).sum() * RES * RES
    report = dict(
        reachable_m2=round(float(reach.sum() * RES * RES), 2),
        stranded_m2=round(float(stranded), 2),
        markers=[],
    )
    if stranded > 0.05:
        problems.append(f'{stranded:.2f} m^2 of free space cannot be reached by a {ROBOT_RADIUS} m robot')
    # comfortable reach with the nav2 default-ish 0.30 m clearance, to flag tight spots
    # Same flood fill with fatter robots: shows how much floor sits behind tight gaps.
    reach_comfy = ras.flood(ras.inflate(ras.occ, 0.30), w['spawn'][:2])
    report['robot_centre_floor_m2_by_clearance'] = {
        f'{c:.2f}': round(float(ras.flood(ras.inflate(ras.occ, c), w['spawn'][:2]).sum() * RES * RES), 2)
        for c in (0.22, 0.30, 0.35, 0.39, 0.45)}
    report['markers_readable_by_clearance'] = {}
    view_maps = {}
    step = 2
    for m in w['markers']:
        centre = marker_frame(m)[0]
        vis = np.zeros_like(reach)
        blockers = ras.sight_blockers(m['id'])
        best = None
        for i in range(0, ras.nx, step):
            for j in range(0, ras.ny, step):
                if not reach[i, j]:
                    continue
                px, py = ras.X[i, j], ras.Y[i, j]
                d = math.hypot(px - centre[0], py - centre[1])
                if d > 3.0:
                    continue
                if readable_from(m, ras, blockers, px, py):
                    vis[i, j] = True
                    if best is None or d > best:
                        best = d
        area = float(vis.sum() * (RES * step) ** 2)
        view_maps[m['id']] = vis
        for c in (0.30, 0.35, 0.39, 0.45):
            r = ras.flood(ras.inflate(ras.occ, c), w['spawn'][:2])
            report['markers_readable_by_clearance'].setdefault(f'{c:.2f}', []).append(
                m['id']) if (vis & r).any() else None
        report['markers'].append(dict(id=m['id'], x=m['x'], y=m['y'], z=m['z'],
                                      tag_centre=[round(float(v), 3) for v in centre],
                                      tag_normal=[round(float(v), 3) for v in marker_frame(m)[1]],
                                      tag_centre_in_spawn_frame=in_spawn_frame(w, centre),
                                      readable_with_0p30_clearance=bool((vis & reach_comfy).any()),
                                      readable_area_m2=round(area, 2),
                                      max_read_range_m=round(best, 2) if best else None,
                                      note=m['note']))
        if area < 0.05:
            problems.append(f"tag {m['id']} cannot be read from anywhere the robot can stand")
    ids = [m['id'] for m in w['markers']]
    if len(set(ids)) != len(ids):
        problems.append('duplicate tag IDs')
    return report, problems, ras, reach, view_maps


# --------------------------------------------------------------------------
# Preview images
# --------------------------------------------------------------------------
def preview(name, w, ras, reach, view_maps, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle

    fig, ax = plt.subplots(figsize=(8, 8.4), dpi=110)
    ext = [ras.x0, ras.x0 + ras.nx * RES, ras.y0, ras.y0 + ras.ny * RES]
    colours = plt.cm.tab10(np.linspace(0, 1, 10))
    for k, m in enumerate(w['markers']):
        rgba = np.zeros((ras.ny, ras.nx, 4))
        rgba[..., :3] = colours[k % 10][:3]
        rgba[..., 3] = view_maps[m['id']].T * 0.35
        ax.imshow(rgba, extent=ext, origin='lower', interpolation='nearest', zorder=1)
    for cx, cy, sx, sy in wall_rectangles(w):
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color='black', zorder=2))
    for e in w['extras']:
        if e['kind'] == 'cyl':
            ax.add_patch(Circle((e['cx'], e['cy']), e['r'], color='black', zorder=2))
        else:
            c, s = math.cos(e['yaw']), math.sin(e['yaw'])
            pts = [(e['cx'] + c * a - s * b, e['cy'] + s * a + c * b)
                   for a, b in ((-e['sx'] / 2, -e['sy'] / 2), (e['sx'] / 2, -e['sy'] / 2),
                                (e['sx'] / 2, e['sy'] / 2), (-e['sx'] / 2, e['sy'] / 2))]
            ax.add_patch(Polygon(pts, color='black', zorder=2))
    for k, m in enumerate(w['markers']):
        centre, normal, _, _ = marker_frame(m)
        col = colours[k % 10]
        ax.add_patch(Rectangle((m['x'] - 0.09, m['y'] - 0.09), 0.18, 0.18, color=col, zorder=4))
        ax.annotate('', xy=(centre[0] + 0.55 * normal[0], centre[1] + 0.55 * normal[1]),
                    xytext=(centre[0], centre[1]), zorder=5,
                    arrowprops=dict(arrowstyle='->', color=col, lw=2))
        side = marker_frame(m)[2]
        ax.text(centre[0] + 0.8 * normal[0] + m['label_shift'] * side[0],
                centre[1] + 0.8 * normal[1] + m['label_shift'] * side[1], str(m['id']), color=col,
                fontsize=12, fontweight='bold', ha='center', va='center', zorder=6,
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec=col, lw=1))
    sx, sy, syaw = w['spawn']
    ax.add_patch(Circle((sx, sy), ROBOT_RADIUS, fc='limegreen', ec='darkgreen', zorder=4))
    ax.annotate('', xy=(sx + 0.6 * math.cos(syaw), sy + 0.6 * math.sin(syaw)), xytext=(sx, sy),
                arrowprops=dict(arrowstyle='->', color='darkgreen', lw=2.5), zorder=5)
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect('equal')
    ax.set_xticks(range(-5, 6))
    ax.set_yticks(range(-5, 6))
    ax.grid(True, color='0.85', lw=0.5, zorder=0)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f"{w['title']}  ({name}.launch.py)\n"
                 'green = spawn pose, numbers = tag IDs, tint = where that tag is readable from',
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def make_textures(ids, tex_dir):
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    for i in ids:
        img = cv2.aruco.generateImageMarker(dictionary, i, 354)  # 354 px, no quiet zone: same as reference
        cv2.imwrite(os.path.join(tex_dir, f'tw_6x6_1000-{i}.png'), cv2.cvtColor(img, cv2.COLOR_GRAY2BGRA))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--previews', action='store_true', help='also write docs/<world>.png')
    ap.add_argument('--textures', action='store_true', help='also regenerate the ArUco PNG textures')
    args = ap.parse_args()

    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for d in ('worlds', 'launch', 'docs', 'resource/media/materials/scripts', 'resource/media/materials/textures'):
        os.makedirs(os.path.join(pkg, d), exist_ok=True)

    all_ids = sorted({m['id'] for w in WORLDS.values() for m in w['markers']})
    with open(os.path.join(pkg, 'resource/media/materials/scripts/tw_aruco.material'), 'w', newline='\n') as f:
        f.write(material_script(all_ids))
    if args.textures:
        make_textures(all_ids, os.path.join(pkg, 'resource/media/materials/textures'))

    failed = False
    summary = {}
    for name, w in WORLDS.items():
        for suffix, flag in (('', True), ('_no_aruco', False)):
            with open(os.path.join(pkg, 'worlds', f'{name}{suffix}.world'), 'w', newline='\n') as f:
                f.write(world_sdf(w, flag))
        with open(os.path.join(pkg, 'launch', f'{name}.launch.py'), 'w', newline='\n') as f:
            f.write(launch_file(name, w))

        report, problems, ras, reach, view_maps = validate(name, w)
        summary[name] = dict(spawn=dict(x=w['spawn'][0], y=w['spawn'][1], yaw=round(w['spawn'][2], 4)), **report)
        print(f"\n== {name}: reachable {report['reachable_m2']} m^2, stranded {report['stranded_m2']} m^2")
        for m in report['markers']:
            print(f"   tag {m['id']:>3} at ({m['x']:+.2f}, {m['y']:+.2f}, z={m['z']:.2f})  "
                  f"readable from {m['readable_area_m2']:5.2f} m^2, up to {m['max_read_range_m']} m  | {m['note']}")
        for p in problems:
            failed = True
            print('   PROBLEM:', p)
        if args.previews:
            preview(name, w, ras, reach, view_maps, os.path.join(pkg, 'docs', f'{name}.png'))

    with open(os.path.join(pkg, 'docs', 'ground_truth.json'), 'w', newline='\n') as f:
        json.dump(summary, f, indent=2)
        f.write('\n')
    with open(os.path.join(pkg, 'docs', 'ground_truth.md'), 'w', newline='\n') as f:
        f.write(ground_truth_md(summary))
    print('\nFAILED' if failed else '\nall worlds OK')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
