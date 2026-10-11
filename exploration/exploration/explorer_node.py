"""
explorer_node.py

The single ROS node for autonomous frontier exploration. It talks to ROS on both
ends and delegates all the actual computation.

The node owns all state (latest map, robot pose lookup, blacklist, "are we
navigating?"). The logic modules are stateless and receive that state as
arguments — they know nothing about ROS.

Follows the Prac 4 waypoint_cycler pattern: publish a PoseStamped to /goal_pose,
and listen to /behavior_tree_log to know when the current navigation has finished
so the next goal can be chosen.
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog

import tf2_ros
from tf2_ros import TransformException

from exploration.frontier_detection import find_frontiers
from exploration.waypoint_finder import WaypointFinder
from exploration.waypoint_decider import WaypointDecider
from exploration.path_distance import flood_distances
from visualization_msgs.msg import Marker, MarkerArray


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ---- tuning knobs ----
        self.sensor_range_m = 3.5      # [m] still used for cluster sizing
        self.gain_radius_m = 1.5       # [m] unknown counted only this close to a candidate
        self.a = 1.0                   # utility per m^2 of new map
        self.b = 1.0                   # utility per metre of walking
        self.goal_reached_dist = 0.5   # [m] robot this close to goal counts as reached

        # ---- state (owned by the node) ----
        self.latest_map = None         # most recent OccupancyGrid
        self.current_goal = None       # (x, y) position we are currently driving to
        self.navigating = False        # are we mid-navigation right now?
        self.blacklist = []            # (x, y) goals that failed

        self.goal_timeout_s = 60.0     # max time to spend on one goal
        self.goal_sent_time = None     # time to when goal was published
        self.create_timer(1.0, self.check_timeout)
        
        # ---- tf2 for robot pose lookup (map -> base_link) ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- ROS interfaces ----
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # --- Nav2 occupancy grid ---
        self.latest_costmap = None
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.max_goal_cost = 80        # 0..100; ~0.3 m from a wall with default inflation

        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self.bt_log_callback, 10)

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

        self.get_logger().info('Frontier explorer started.')

        # when turning at goal
        self.hold_s = 2.0              # [s] sit still after arriving so the camera gets frames
        self.hold_timer = None

        # ---- marker targeting ----
        self.pending_markers = {}      # id -> (x, y, yaw from marker toward viewer)
        self.visited_markers = set()   # ids we have already driven to look at
        self.marker_standoff_m = 0.8   # [m] how far in front of the marker to stand
        self.marker_hold_s = 3.0       # [s] longer pause for markers (needs ~8 frames)
        self.goal_kind = None          # 'frontier' or 'marker'
        self.current_marker_id = None
        self.create_subscription(MarkerArray, '/aruco/unconfirmed', self.unconfirmed_callback, 10)

        
    # ---- /map: upstaes latest map, if idle we start ecploring -----------------

    def map_callback(self, msg):
        self.latest_map = msg

        if not self.navigating:
            self.plan_and_send_goal()

    # ---- /behavior_tree_log: when navigation finished -> plan the next goal -------

    def bt_log_callback(self, msg):
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                self.on_navigation_finished()

    def on_navigation_finished(self):
        if self.hold_timer is not None:
            return                      # already holding; Nav2 can report IDLE twice

        if self.goal_kind == 'marker':
            self.visited_markers.add(self.current_marker_id)
            self.get_logger().info(f'Marker {self.current_marker_id}: looked, moving on')
        elif self.current_goal is not None:
            robot = self.lookup_robot_pose()
            if robot is not None:
                robot_x, robot_y = robot
                goal_x, goal_y = self.current_goal
                dist = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)
                if dist > self.goal_reached_dist:
                    self.get_logger().info(f'Goal failed, blacklisted: {self.current_goal}')
            self.blacklist.append(self.current_goal)

        # Stay "navigating" during the hold so map_callback does not plan early.
        hold = self.marker_hold_s if self.goal_kind == 'marker' else self.hold_s
        self.goal_sent_time = None
        self.hold_timer = self.create_timer(hold, self._hold_done)

    def _hold_done(self):
        self.destroy_timer(self.hold_timer)
        self.hold_timer = None
        self.navigating = False
        self.plan_and_send_goal()

    def check_timeout(self):
        # see how long a goal has been a target
        if not self.navigating or self.goal_sent_time is None:
            return
        elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds * 1e-9
        
        # remove goals that timeout
        if elapsed > self.goal_timeout_s:
            if self.goal_kind == 'marker':
                self.get_logger().warn(
                    f'Marker {self.current_marker_id}: viewpoint unreachable, skipping')
                self.visited_markers.add(self.current_marker_id)
            else:
                self.get_logger().warn(
                    f'Goal {self.current_goal} timed out after {elapsed:.0f}s, blacklisting')
                self.blacklist.append(self.current_goal)
            self.navigating = False
            self.goal_sent_time = None
            self.plan_and_send_goal()

    # ---- the core: frontiers -> finder -> decider -> goal -------------------

    def plan_and_send_goal(self):
        # 0: an unconfirmed marker beats any frontier. Visit each one once.
        target = self.pick_marker_target()
        if target is not None:
            mid, x, y, yaw = target
            self.get_logger().info(
                f'Marker {mid} unconfirmed, going to look from ({x:.2f}, {y:.2f})')
            self.publish_goal(x, y, yaw)
            self.current_goal = (x, y)
            self.current_marker_id = mid
            self.goal_kind = 'marker'
            self.navigating = True
            self.goal_sent_time = self.get_clock().now()
            return
        
        if self.latest_map is None:
            return

        msg = self.latest_map
        info = msg.info
        data = msg.data

        # 1: Detect frontier cells (pure logic module).
        frontier_cells = find_frontiers(data, info.width, info.height)
        if not frontier_cells:
            self.get_logger().info('No frontiers left — exploration complete.')
            return

        # 2: Size knobs derived from map resolution + sensor range.
        resolution = info.resolution
        gain_radius_cells = int(round(self.gain_radius_m / resolution))
        max_cluster_size = int(round((0.5 * self.sensor_range_m) / resolution))

        finder = WaypointFinder(
            max_cluster_size=max_cluster_size,
            min_cluster_size=5,
            sensor_radius_cells=gain_radius_cells,
        )

        waypoints = finder.find_waypoints(data, info, frontier_cells)
        if not waypoints:
            self.get_logger().info('No usable waypoints this cycle.')
            return

         # 2b: slide every candidate out of the inflation ring and drop hopeless ones.
        usable = []
        for wp in waypoints:
            moved = self.nudge_to_low_cost(wp.x, wp.y)
            if moved is None:
                continue
            wp.x, wp.y = moved
            # keep the grid cell in step with the new x, y (6b looks cells up)
            wp.col = int((wp.x - info.origin.position.x) / resolution)
            wp.row = int((wp.y - info.origin.position.y) / resolution)
            usable.append(wp)
        if not usable:
            self.get_logger().info('All waypoints inside the inflation ring this cycle.')
            return
        
        waypoints = usable

        # 3: Where is the robot right now?
        robot = self.lookup_robot_pose()
        if robot is None:
            self.get_logger().warn('No robot pose yet — will retry next map.')
            return
        robot_x, robot_y = robot

        # 3b: walking distance from the robot to every free cell.
        r_col = int((robot_x - info.origin.position.x) / resolution)
        r_row = int((robot_y - info.origin.position.y) / resolution)
        path_dists = flood_distances(data, info.width, info.height, resolution,
                                     (r_row, r_col))
        if not path_dists:
            self.get_logger().warn('Robot cell not free in /map, using straight-line cost.')
            path_dists = None

        # 4: Score candidates and pick the best.
        decider = WaypointDecider(a=self.a, b=self.b)
        best, utility = decider.choose(
            waypoints, robot_x, robot_y, self.blacklist, path_dists)

        if best is None and self.blacklist:
            # Every candidate sits inside a blacklist circle. The list is only a
            # hint, so wipe it and re-score; truly bad goals will fail again.
            self.get_logger().warn(
                f'All {len(waypoints)} waypoints blacklisted — clearing '
                f'{len(self.blacklist)} entries and retrying.')
            self.blacklist = []
            best, utility = decider.choose(
                waypoints, robot_x, robot_y, self.blacklist, path_dists)

        if best is None:
            self.get_logger().info('No usable waypoint after clearing blacklist — nothing to do.')
            return

        # log chosen goal
        straight = math.hypot(best.x - robot_x, best.y - robot_y)
        walk = path_dists.get((best.row, best.col)) if path_dists else straight
        self.get_logger().info(
            f'Chosen goal ({best.x:.2f}, {best.y:.2f}) gain={best.gain:.1f}m2 '
            f'walk={walk:.1f}m straight={straight:.1f}m utility={utility:.1f}')

        # 5) Send it to Nav2.
        yaw = best.yaw
        if yaw is None:
            yaw = math.atan2(best.y - robot_y, best.x - robot_x)   # face direction of travel
        self.publish_goal(best.x, best.y, yaw)
        self.current_goal = (best.x, best.y)
        self.goal_kind = 'frontier'
        self.navigating = True
        self.goal_sent_time = self.get_clock().now()

    # ---- helpers ------------------------------------------------------------

    def lookup_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        return (x, y)

    def costmap_callback(self, msg):
        self.latest_costmap = msg

    def costmap_cost(self, x, y):
        """Cost 0..100 at world (x, y), or None if no costmap / outside it."""
        cm = self.latest_costmap
        if cm is None:
            return None
        info = cm.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return None
        return cm.data[row * info.width + col]

    def nudge_to_low_cost(self, x, y, max_radius_m=0.5):
        """
        Nearest (x, y) whose cost is <= max_goal_cost, searching a square
        up to max_radius_m wide. None if the whole neighbourhood is costly.
        """
        cm = self.latest_costmap
        if cm is None:
            return (x, y)                       # no costmap yet: trust the map
        res = cm.info.resolution
        steps = int(round(max_radius_m / res))
        best = None
        best_d2 = None
        for d_row in range(-steps, steps + 1):
            for d_col in range(-steps, steps + 1):
                nx = x + d_col * res
                ny = y + d_row * res
                c = self.costmap_cost(nx, ny)
                if c is None or c < 0 or c > self.max_goal_cost:
                    continue
                if not self.map_is_free(nx, ny):
                    continue                        # flood (6b) must be able to reach it
                d2 = d_row * d_row + d_col * d_col
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best = (nx, ny)
        return best

    def publish_goal(self, x, y, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)

    def unconfirmed_callback(self, msg):
        pending = {}
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                continue
            q = m.pose.orientation
            yaw = 2.0 * math.atan2(q.z, q.w)          # inverse of z=sin(yaw/2), w=cos(yaw/2)
            pending[m.id] = (m.pose.position.x, m.pose.position.y, yaw)
        self.pending_markers = pending

    def pick_marker_target(self):
        """(id, x, y, yaw) for the first unvisited unconfirmed marker, else None."""
        for mid, (mx, my, view_yaw) in sorted(self.pending_markers.items()):
            if mid in self.visited_markers:
                continue
            vx = mx + self.marker_standoff_m * math.cos(view_yaw)
            vy = my + self.marker_standoff_m * math.sin(view_yaw)
            moved = self.nudge_to_low_cost(vx, vy)     # from 6a
            if moved is None:
                self.get_logger().warn(f'Marker {mid}: no free viewpoint, skipping')
                self.visited_markers.add(mid)
                continue
            vx, vy = moved
            face_yaw = math.atan2(my - vy, mx - vx)
            return mid, vx, vy, face_yaw
        return None

    def map_is_free(self, x, y):
        """True if /map says world (x, y) is a known free cell."""
        m = self.latest_map
        if m is None:
            return True
        info = m.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return False
        return m.data[row * info.width + col] == 0


def main(args=None):
    rclpy.init(args=args)
    frontier_explorer = FrontierExplorer()
    rclpy.spin(frontier_explorer)
    frontier_explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
