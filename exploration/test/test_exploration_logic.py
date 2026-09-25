"""
test_exploration_logic.py

ROS-free unit tests for the three logic modules. Because none of them import
rclpy, we can build small hand-made occupancy grids and check that the right
values come out — no simulator, no Nav2, no tf2 required.

Maps are written as lists of strings for readability:
    '.'  free      -> 0
    '?'  unknown   -> -1
    '#'  occupied  -> 100

Run with:
    colcon test --packages-select exploration
    colcon test-result --verbose
or, from the package directory with the workspace sourced:
    python -m pytest test/test_exploration_logic.py -v
"""

from exploration.frontier_detection import find_frontiers
from exploration.waypoint_finder import WaypointFinder, Waypoint
from exploration.waypoint_decider import WaypointDecider


# ---- helpers ------------------------------------------------------------

def build_grid(rows):
    """Turn a list of strings into (data, width, height) like a ROS map."""
    height = len(rows)
    width = len(rows[0])

    data = []
    for r in range(height):
        for c in range(width):
            ch = rows[r][c]
            if ch == '.':
                data.append(0)      # free
            elif ch == '?':
                data.append(-1)     # unknown
            else:
                data.append(100)    # occupied
    return data, width, height


class FakePosition:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class FakeOrigin:
    def __init__(self, x, y):
        self.position = FakePosition(x, y)


class FakeMapInfo:
    """Stand-in for msg.info — only the fields the finder actually reads."""

    def __init__(self, width, height, resolution,
                 origin_x=0.0, origin_y=0.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = FakeOrigin(origin_x, origin_y)


# ---- frontier detection -------------------------------------------------

def test_frontiers_lie_on_the_free_unknown_boundary():
    # Left half free, right half unknown. The only free cells that touch
    # unknown are in column 4, so the frontier is that vertical line.
    data, width, height = build_grid([
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
    ])

    frontiers = find_frontiers(data, width, height)

    assert len(frontiers) == 10
    for (row, col) in frontiers:
        assert col == 4


def test_no_frontiers_in_fully_known_map():
    # No unknown cells anywhere -> no frontier.
    data, width, height = build_grid([
        '.....',
        '.....',
        '.....',
    ])

    frontiers = find_frontiers(data, width, height)

    assert frontiers == []


# ---- clustering (via the finder) ----------------------------------------

def test_large_frontier_stays_one_cluster_when_cap_is_high():
    # The 10-cell vertical frontier fits under a high cap, so it is not split.
    data, width, height = build_grid([
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
    ])
    info = FakeMapInfo(width, height, resolution=0.05)
    frontiers = find_frontiers(data, width, height)

    finder = WaypointFinder(
        max_cluster_size=100,     # far bigger than the frontier
        min_cluster_size=1,
        sensor_radius_cells=3,
    )
    waypoints = finder.find_waypoints(data, info, frontiers)

    assert len(waypoints) == 1


def test_long_frontier_is_split_when_cap_is_low():
    # Same 10-cell frontier, but a cap of 5 forces it into two bites.
    data, width, height = build_grid([
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
        '.....?????',
    ])
    info = FakeMapInfo(width, height, resolution=0.05)
    frontiers = find_frontiers(data, width, height)

    finder = WaypointFinder(
        max_cluster_size=5,       # forces a split of the 10-cell line
        min_cluster_size=1,
        sensor_radius_cells=3,
    )
    waypoints = finder.find_waypoints(data, info, frontiers)

    assert len(waypoints) == 2


def test_tiny_frontier_is_rejected_as_noise():
    # A single unknown cell in the corner creates only a 2-cell frontier.
    # With min_cluster_size = 5 it is discarded as noise.
    data, width, height = build_grid([
        '?....',
        '.....',
        '.....',
        '.....',
        '.....',
    ])
    info = FakeMapInfo(width, height, resolution=0.05)
    frontiers = find_frontiers(data, width, height)

    finder = WaypointFinder(
        max_cluster_size=100,
        min_cluster_size=5,       # bigger than the 2-cell frontier
        sensor_radius_cells=3,
    )
    waypoints = finder.find_waypoints(data, info, frontiers)

    assert len(waypoints) == 0


# ---- gain estimate ------------------------------------------------------

def test_gain_counts_unknown_and_is_zero_with_none_nearby():
    finder = WaypointFinder(
        max_cluster_size=100,
        min_cluster_size=1,
        sensor_radius_cells=4,
    )

    # Map A: a big unknown region sits right next to the waypoint.
    data_a, width_a, height_a = build_grid([
        '......?????',
        '......?????',
        '......?????',
        '......?????',
        '......?????',
        '......?????',
        '......?????',
        '......?????',
        '......?????',
        '......?????',
        '......?????',
    ])
    gain_a = finder._estimate_gain(data_a, width_a, height_a, (5, 5))

    # Map B: fully known, so nothing to reveal.
    data_b, width_b, height_b = build_grid([
        '...........',
        '...........',
        '...........',
        '...........',
        '...........',
        '...........',
        '...........',
        '...........',
        '...........',
        '...........',
        '...........',
    ])
    gain_b = finder._estimate_gain(data_b, width_b, height_b, (5, 5))

    assert gain_a > 0
    assert gain_b == 0


# ---- grid -> world conversion -------------------------------------------

def test_grid_to_world_uses_cell_centre_and_origin():
    finder = WaypointFinder(
        max_cluster_size=100,
        min_cluster_size=1,
        sensor_radius_cells=3,
    )
    info = FakeMapInfo(width=10, height=10, resolution=0.5,
                       origin_x=1.0, origin_y=-2.0)

    # Cell (row=2, col=3):
    #   x = 1.0 + (3 + 0.5) * 0.5 =  2.75
    #   y = -2.0 + (2 + 0.5) * 0.5 = -0.75
    x, y = finder._grid_to_world(2, 3, info)

    assert abs(x - 2.75) < 1e-9
    assert abs(y - (-0.75)) < 1e-9


# ---- the decider --------------------------------------------------------

def make_waypoint(x, y, gain):
    # row/col/cluster_size are irrelevant to the decider; fill with dummies.
    return Waypoint(row=0, col=0, x=x, y=y, gain=gain, cluster_size=1)


def test_decider_prefers_higher_utility():
    robot_x, robot_y = 0.0, 0.0
    # A: lots of gain a bit far. B: little gain, close.
    a_wp = make_waypoint(x=10.0, y=0.0, gain=100)
    b_wp = make_waypoint(x=1.0, y=0.0, gain=10)

    decider = WaypointDecider(a=1.0, b=1.0)
    best, utility = decider.choose([a_wp, b_wp], robot_x, robot_y, blacklist=[])

    # U_A = 100 - 10 = 90 ; U_B = 10 - 1 = 9  ->  A wins.
    assert best is a_wp


def test_decider_skips_blacklisted_goals():
    robot_x, robot_y = 0.0, 0.0
    bad = make_waypoint(x=5.0, y=0.0, gain=100)   # high gain but blacklisted
    ok = make_waypoint(x=2.0, y=0.0, gain=10)

    decider = WaypointDecider(a=1.0, b=1.0, blacklist_radius=0.5)
    best, utility = decider.choose(
        [bad, ok], robot_x, robot_y, blacklist=[(5.0, 0.0)])

    assert best is ok


def test_decider_penalises_goals_on_top_of_the_robot():
    robot_x, robot_y = 0.0, 0.0
    # Close waypoint has more raw gain, but sits inside min_travel_dist,
    # so the jitter penalty should knock it out in favour of the far one.
    close_wp = make_waypoint(x=0.1, y=0.0, gain=50)
    far_wp = make_waypoint(x=2.0, y=0.0, gain=10)

    decider = WaypointDecider(
        a=1.0, b=1.0, min_travel_dist=0.3, close_penalty=1000.0)
    best, utility = decider.choose(
        [close_wp, far_wp], robot_x, robot_y, blacklist=[])

    assert best is far_wp


def test_decider_returns_none_when_no_candidates():
    decider = WaypointDecider(a=1.0, b=1.0)
    best, utility = decider.choose([], 0.0, 0.0, blacklist=[])

    assert best is None
    assert utility is None


def test_decider_returns_none_when_all_blacklisted():
    only = make_waypoint(x=3.0, y=0.0, gain=100)

    decider = WaypointDecider(a=1.0, b=1.0, blacklist_radius=0.5)
    best, utility = decider.choose(
        [only], 0.0, 0.0, blacklist=[(3.0, 0.0)])

    assert best is None
    assert utility is None
