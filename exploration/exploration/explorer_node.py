"""
explorer_node.py

The single ROS node for autonomous frontier exploration. It talks to ROS on both
ends and delegates all the actual computation.

The node owns all state (latest map, robot pose lookup, blacklist, "are we
navigating?"). The logic modules are stateless and receive that state as
arguments, so they know nothing about ROS.

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
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException

from exploration.frontier_detection import find_frontiers
from exploration.waypoint_finder import WaypointFinder
from exploration.waypoint_decider import WaypointDecider


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ---- tuning knobs ----
        self.sensor_range_m = 3.5      # LDS-01 lidar range on the Waffle Pi
        self.gain_radius_m = 1.5       # only count unknown cells this close to a candidate
        self.a = 1.0                   # utility weight on information gain
        self.b = 1.0                   # utility weight on travel cost
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

        # Nav2's costmap, so we can keep goals out of the inflated ring around walls
        self.latest_costmap = None
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.max_goal_cost = 80        # 0..100, anything higher is too close to a wall

        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self.bt_log_callback, 10)

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

        self.marker_sub = self.create_subscription(
            MarkerArray, '/aruco/map_markers', self.flag_marker, 10)


        self.seen_markers = {}
        self.inspected_markers = set()
        self.current_marker_id = None
        
        self.get_logger().info('Frontier explorer started.')

        # when turning at goal
        self.hold_s = 2.0              # [s] sit still after arriving so the camera gets frames
        self.hold_timer = None

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

        # Check if robot reached goal, and start planning next goal
        if self.current_goal is not None:
            robot = self.lookup_robot_pose()
            if robot is not None:
                robot_x, robot_y = robot
                goal_x, goal_y = self.current_goal
                dist = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)
                if dist > self.goal_reached_dist:
                    self.blacklist.append(self.current_goal)
                    self.get_logger().info(
                        f'Goal failed, blacklisted: {self.current_goal}')

        # Stay "navigating" during the hold so map_callback does not plan early.
        self.goal_sent_time = None
        self.hold_timer = self.create_timer(self.hold_s, self._hold_done)

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
            self.get_logger().warn(
                f'Goal {self.current_goal} timed out after {elapsed:.0f}s, blacklisting')
            self.blacklist.append(self.current_goal)
            self.navigating = False
            self.goal_sent_time = None
            self.plan_and_send_goal()

    # ---- the core: frontiers -> finder -> decider -> goal -------------------

    def plan_and_send_goal(self):
        if self.latest_map is None:
            return

        msg = self.latest_map
        info = msg.info
        data = msg.data

        # Find robot position
        robot = self.lookup_robot_pose()
        if robot is None:
            self.get_logger().warn('No robot pose yet, will retry next map.')
            return
        robot_x, robot_y = robot

        waypoints = []

        # 1: Detect frontier cells 
        frontier_cells = find_frontiers(data, info.width, info.height)

        # 2: Find waypoints from frontier cells
        if frontier_cells:
            resolution = info.resolution
            gain_radius_cells = int(round(self.gain_radius_m / resolution))
            max_cluster_size = int(round((0.5 * self.sensor_range_m) / resolution))

            finder = WaypointFinder(
                max_cluster_size=max_cluster_size,
                min_cluster_size=5,
                sensor_radius_cells=gain_radius_cells,
            )
            
            for wp in finder.find_waypoints(data, info, frontier_cells):
                moved = self.nudge_to_low_cost(wp.x, wp.y)
                if moved is None:
                    continue
                wp.x, wp.y = moved
                waypoints.append(wp)

        # 3: Find waypoints from markers
        for marker_id in self.seen_markers:
            if marker_in in self.inspected_markers:
                continue
            mx, my = self.seen_markers[marker_id]
            moved = self.nudge_to_low_cost(mx, my, max_radius_m=1.0, max_cost=30, robot_pos=(robot_x, robot_y))

            if moved is None:
                continue
            gx, gy = moved
            waypoints.append(Waypoint(None, None, gx, gy, gain = 0.0, cluster_size = 0, marker_id = marker_id, marker_x = mx, marker_y = my))

        if not waypoints:
            self.get_logger().info("No frontier or markers left, exploration complete!")
            return
        
        # 4: Score candidates and pick the best.
        decider = WaypointDecider(a=self.a, b=self.b)
        best, utility = decider.choose(
            waypoints, robot_x, robot_y, self.blacklist)

        if best is None and self.blacklist:
            # everything is inside a blacklist circle, so clear it and score again
            self.get_logger().warn(
                f'All {len(waypoints)} waypoints blacklisted, clearing '
                f'{len(self.blacklist)} entries and retrying.')
            self.blacklist = []
            best, utility = decider.choose(
                waypoints, robot_x, robot_y, self.blacklist)

        if best is None:
            self.get_logger().info('No usable waypoint after clearing blacklist, nothing to do.')
            return

        self.get_logger().info(
            f'Chosen goal ({best.x:.2f}, {best.y:.2f}) '
            f'gain={best.gain:.1f} utility={utility:.1f}')

        # 5: Send best waypoint to Nav2
        if best.marker_id is not None:
            yaw = math.atan2(best.marker_y - best.y, best.marker_x - best.x)   # face direction of marker
        else:
            yaw = math.atan2(best.y - robot.y, best.x - robot.x) # face direction of travel
            
        self.publish_goal(best.x, best.y, yaw)
        self.current_goal = (best.x, best.y)
        self.current_marker_id = best.marker_id 
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

    def nudge_to_low_cost(self, wp_x, wp_y, max_radius_m=0.5, max_cost=None, robot_pos=None):
        """
        Nearest (x, y) whose cost is <= max_goal_cost, searching a square
        up to max_radius_m wide. None if the whole neighbourhood is costly.
        """
        
        if max_cost is None:
            max_cost = self.max_goal_cost
            
        cm = self.latest_costmap
        if cm is None:
            return (wp_x, wp_y)                       # no costmap yet: trust the map
            
        res = cm.info.resolution
        steps = int(round(max_radius_m / res))
        best = None
        best_d2 = None
        
        for d_row in range(-steps, steps + 1):
            for d_col in range(-steps, steps + 1):
                nx = wp_x + d_col * res
                ny = wp_y + d_row * res
                c = self.costmap_cost(nx, ny)
                
                if c is None or c < 0 or c > self.max_goal_cost:
                    continue

                if robot_pos is not None:
                    # k = candidate - waypoint, r = robot - waypoint
                    k_x = nx - wp_x
                    k_y = ky - wp_y
                    r_x = robot_pos[0] - wp_x
                    r_y = robot_pos[1] - wp_y
                    
                    if k_x * rx + k_y * ry <= 0: # Wrong side of the wall
                        continue
                    
                d2 = d_row * d_row + d_col * d_col
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best = (nx, ny)
        return best

    def flag_marker(self, msg):
        
        for marker in msg.markers:
            marker_id = marker.id
            x = marker.pose.position.x
            y = marker.pose.position.y

            # update position with each new publication
            self.seen_markers[marker_id] = (x,y)

    def publish_goal(self, x, y, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    frontier_explorer = FrontierExplorer()
    rclpy.spin(frontier_explorer)
    frontier_explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
