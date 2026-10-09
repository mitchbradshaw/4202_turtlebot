"""
waypoint_decider.py

Scores the candidate waypoints from WaypointFinder and returns the best one.

Utility per candidate:
    U = a * gain  -  b * cost  -  penalties

    gain      : from the finder (unknown cells reachable from the waypoint)
    cost      : straight-line distance from the robot to the waypoint [m]
    penalties : blacklist (failed goals) + a too-close term to stop jitter

The class is pure logic. The owning node supplies the robot pose (from tf2)
and the current blacklist every cycle.
"""

import math


class WaypointDecider:

    def __init__(self,
                 a,
                 b,
                 blacklist_radius=0.5,
                 min_travel_dist=0.5,
                 close_penalty=1000.0):
        self.a = a                                # weight on information gain
        self.b = b                                # weight on travel cost
        self.blacklist_radius = blacklist_radius  # [m] how near a failed goal counts as "same"
        self.min_travel_dist = min_travel_dist    # [m] below this, apply the jitter penalty
        self.close_penalty = close_penalty        # penalty added for too-close goals

    # ---- public entry point -------------------------------------------------

    def choose(self, waypoints, robot_x, robot_y, blacklist):
        """
        waypoints : list[Waypoint] from WaypointFinder
        robot_x/y : robot position in the map frame [m] (from tf2)
        blacklist : list of (x, y) world points that previously failed
        returns   : (best_waypoint, best_utility), or (None, None) if none are valid
        """
        best_waypoint = None
        best_utility = None

        for wp in waypoints:
            # Hard skip: a goal we already failed to reach.
            if self._is_blacklisted(wp, blacklist):
                continue

            cost = self._distance(robot_x, robot_y, wp.x, wp.y)
            # hard skip goals that are too close
            if cost < self.min_travel_dist:
                continue
            
            penalty = self._penalty(cost)

            utility = self.a * wp.gain - self.b * cost - penalty

            if best_utility is None or utility > best_utility:
                best_utility = utility
                best_waypoint = wp

        return best_waypoint, best_utility

    # ---- helpers ------------------------------------------------------------

    def _distance(self, x1, y1, x2, y2):
        d_x = x2 - x1
        d_y = y2 - y1
        return math.sqrt(d_x * d_x + d_y * d_y)

    def _is_blacklisted(self, wp, blacklist):
        for (b_x, b_y) in blacklist:
            if self._distance(wp.x, wp.y, b_x, b_y) < self.blacklist_radius:
                return True
        return False

    def _penalty(self, cost):
        penalty = 0.0

        # Discourage goals almost on top of the robot: they make the robot twitch
        # between tiny nearby frontiers instead of making real progress.
        if cost < self.min_travel_dist:
            penalty += self.close_penalty

        # Extra penalty terms go here later, e.g. a revisit penalty or a
        # penalty for goals pointing back into already-explored space.

        return penalty
