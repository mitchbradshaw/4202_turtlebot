from frontier_detector import frontiers



def find_cluster(frontier, map, search_radius):

    cluster = []

    radius_cells = int(search_radius / map.resolution)
    for dx in range (-radius_cells, radius_cells+1):
        for dy in range (-radius_cells, radius_cells + 1):

            if dx**2 + dy**2 > radius_cells**2:
                continue
            
            x = frontier.x + dx
            y = frontier.y + dy

            if (x, y) in frontiers:
                cluster.append((x, y))
    return cluster

def best_cluster(frontiers):
    best_cluster = None
    best_score = -infinity

    for frontier in frontiers:
        cluster = find_cluster(frontier, map, search radius)
        score = calculate_score(cluster, robot_position, frontier)

        if score > best_score:
            best_score = score
            best_cluster = cluster

    return best_cluster

def calculate_score(clsuter, robot_position, frontier):
    return cluster_weight * len(cluster) - distance_weight * distance_to_robot

def find_waypoint(best_cluster)


