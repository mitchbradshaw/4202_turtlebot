"""
explorer_node.py

The single ROS node for autonomous frontier exploration. It talks to ROS on both
ends and delegates all the actual computation to three pure logic modules:

    /map  ->  frontier_detection.find_frontiers
          ->  waypoint_finder.find_waypoints
          ->  (tf2: look up robot pose)
          ->  waypoint_decider.choose
          ->  /goal_pose

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
from rclpy.parameter import Parameter

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog

import tf2_ros
from tf2_ros import TransformException

from exploration.frontier_detection import find_frontiers
from exploration.waypoint_finder import WaypointFinder
from exploration.waypoint_decider import WaypointDecider


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # We run against a simulator that publishes /clock, so the node must use
        # sim time or tf2 lookups will silently fail. (You can also pass this at
        # launch with -p use_sim_time:=true.)
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        # ---- tuning knobs ----
        self.sensor_range_m = 3.5      # LDS-01 lidar range on the Waffle Pi
        self.a = 1.0                   # utility weight on information gain
        self.b = 20.0                  # utility weight on travel cost
        self.goal_reached_dist = 0.5   # [m] robot this close to goal counts as reached

        # ---- state (owned by the node) ----
        self.latest_map = None         # most recent OccupancyGrid
        self.current_goal = None       # (x, y) we are currently driving to
        self.navigating = False        # are we mid-navigation right now?
        self.blacklist = []            # (x, y) goals that failed

        # ---- tf2 for robot pose lookup (map -> base_link) ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- ROS interfaces ----
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self.bt_log_callback, 10)

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

        self.get_logger().info('Frontier explorer started.')

    # ---- /map: keep the latest map; kick off the first goal -----------------

    def map_callback(self, msg):
        self.latest_map = msg

        # If we are idle (e.g. just after startup), start exploring. Once we are
        # navigating, further map updates just refresh self.latest_map.
        if not self.navigating:
            self.plan_and_send_goal()

    # ---- /behavior_tree_log: navigation finished -> plan the next goal -------

    def bt_log_callback(self, msg):
        for event in msg.event_log:
            # NavigateRecovery is the root of the default Nav2 BT in Humble. When
            # it returns to IDLE the current navigation has ended (success OR
            # give-up). If goals stop chaining, confirm this node name against
            # your own behaviour-tree XML.
            if event.node_name == 'NavigateRecovery' and \
               event.current_status == 'IDLE':
                self.on_navigation_finished()

    def on_navigation_finished(self):
        # bt_log IDLE fires for both success and failure, so tell them apart by
        # how close the robot ended up to the goal. Stopped far away = the goal
        # was unreachable, so blacklist it.
        if self.current_goal is not None:
            robot = self.lookup_robot_pose()
            if robot is not None:
                robot_x, robot_y = robot
                g_x, g_y = self.current_goal
                dist = math.sqrt((g_x - robot_x) ** 2 + (g_y - robot_y) ** 2)
                if dist > self.goal_reached_dist:
                    self.blacklist.append(self.current_goal)
                    self.get_logger().info(
                        f'Goal failed, blacklisted: {self.current_goal}')

        self.navigating = False
        self.plan_and_send_goal()

    # ---- the core: frontiers -> finder -> decider -> goal -------------------

    def plan_and_send_goal(self):
        if self.latest_map is None:
            return

        msg = self.latest_map
        info = msg.info
        data = msg.data

        # 1) Detect frontier cells (pure logic module).
        frontier_cells = find_frontiers(data, info.width, info.height)
        if not frontier_cells:
            self.get_logger().info('No frontiers left — exploration complete.')
            return

        # 2) Size knobs derived from map resolution + sensor range.
        resolution = info.resolution
        sensor_radius_cells = int(round(self.sensor_range_m / resolution))
        max_cluster_size = int(round((0.5 * self.sensor_range_m) / resolution))

        finder = WaypointFinder(
            max_cluster_size=max_cluster_size,
            min_cluster_size=5,
            sensor_radius_cells=sensor_radius_cells,
        )
        waypoints = finder.find_waypoints(data, info, frontier_cells)
        if not waypoints:
            self.get_logger().info('No usable waypoints this cycle.')
            return

        # 3) Where is the robot right now?
        robot = self.lookup_robot_pose()
        if robot is None:
            self.get_logger().warn('No robot pose yet — will retry next map.')
            return
        robot_x, robot_y = robot

        # 4) Score candidates and pick the best.
        decider = WaypointDecider(a=self.a, b=self.b)
        best, utility = decider.choose(
            waypoints, robot_x, robot_y, self.blacklist)

        if best is None:
            self.get_logger().info('All waypoints blacklisted — nothing to do.')
            return

        self.get_logger().info(
            f'Chosen goal ({best.x:.2f}, {best.y:.2f}) '
            f'gain={best.gain} utility={utility:.1f}')

        # 5) Send it to Nav2.
        self.publish_goal(best.x, best.y)
        self.current_goal = (best.x, best.y)
        self.navigating = True

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

    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        # Identity orientation — Nav2 turns the robot as needed on arrival.
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    frontier_explorer = FrontierExplorer()
    rclpy.spin(frontier_explorer)
    frontier_explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
