"""
Breadth-first flood over free cells of an OccupancyGrid. Gives the walking
distance in metres from the robot to every reachable free cell. No ROS here.
"""
from collections import deque
import math

FREE = 0


def _nearest_free(data, width, height, row, col, max_r=6):
    """Closest FREE cell to (row, col), searching outward. None if none nearby."""
    for r in range(0, max_r + 1):
        for d_row in range(-r, r + 1):
            for d_col in range(-r, r + 1):
                n_row, n_col = row + d_row, col + d_col
                if 0 <= n_row < height and 0 <= n_col < width \
                        and data[n_row * width + n_col] == FREE:
                    return (n_row, n_col)
    return None


def flood_distances(data, width, height, resolution, start):
    """
    data   : msg.data (1D occupancy list)
    start  : (row, col) of the robot. Snaps to the nearest FREE cell because the
             robot's own cell is sometimes marked unknown or occupied.
    returns: dict {(row, col): metres} for every reachable free cell, or {}.
    """
    start = _nearest_free(data, width, height, start[0], start[1])
    if start is None:
        return {}

    diag = math.sqrt(2.0) * resolution
    dist = {start: 0.0}
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        d0 = dist[(row, col)]
        for d_row in (-1, 0, 1):
            for d_col in (-1, 0, 1):
                if d_row == 0 and d_col == 0:
                    continue
                n_row, n_col = row + d_row, col + d_col
                if n_row < 0 or n_row >= height or n_col < 0 or n_col >= width:
                    continue
                if data[n_row * width + n_col] != FREE:
                    continue
                key = (n_row, n_col)
                if key in dist:
                    continue
                step = diag if (d_row and d_col) else resolution
                dist[key] = d0 + step
                queue.append(key)
    return dist