#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt

def inverse_transform_coords(transformed_coords, s):
    return [
        [int(20 * (-transformed_coord[1] + 13.5)), int(20 * (transformed_coord[0] + 23.3))]
        for transformed_coord in transformed_coords
    ]

def generate_random_coords(obstacle_positions, x_range, y_range, n_points):
    x_min, x_max = x_range
    y_min, y_max = y_range

    all_positions = set((x, y) for x in range(x_min, x_max + 1) for y in range(y_min, y_max + 1))
    free_positions = list(all_positions - obstacle_positions)

    if n_points > len(free_positions):
        raise ValueError("Cannot place {} points. Maximum possible points are {}".format(n_points, len(free_positions)))

    return [free_positions[i] for i in np.random.choice(len(free_positions), n_points, replace=False)]


def line(p1, p2):
    x1, y1 = map(int, p1)
    x2, y2 = map(int, p2)

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    is_steep = dy > dx

    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    else:
        swapped = False

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    error = dx // 2
    ystep = 1 if y1 < y2 else -1
    y = y1
    points = []

    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)

        error -= dy
        if error < 0:
            y += ystep
            error += dx

    if swapped:
        points.reverse()

    return list(zip(*points))

def line_of_sight(p1, p2, obstacle_positions):
    x, y = line(p1, p2)  # draw line from p1 to p2
    for i in range(len(x)):
        if (x[i], y[i]) in obstacle_positions:
            return False
    return True
    
def transform_coords(coords, s):
    return np.array([
        [s * coord[1] - 23.3, -s * coord[0] + 13.5]
        for coord in coords
    ])
    
def get_distance(point1, point2):
    return np.linalg.norm(point1 - point2)

def get_path(origin_point, goal_point):
    # Load and preprocess the image
    img = cv2.imread('best_so_far_2.pgm', cv2.IMREAD_GRAYSCALE)
    occupied_thresh = 0.65
    thresh = int(occupied_thresh * 255)
    _, obstacles = cv2.threshold(img, thresh, 255, cv2.THRESH_BINARY_INV)

    # Find the obstacle coordinates
    obstacle_coords = np.column_stack(np.where(obstacles > 0))
    obstacle_positions = set((x, y) for x, y in obstacle_coords)

    # Define coordinate ranges based on obstacle_coords
    x_min, x_max = obstacle_coords[:, 0].min(), obstacle_coords[:, 0].max()
    y_min, y_max = obstacle_coords[:, 1].min(), obstacle_coords[:, 1].max()

    # Generate random points
    n_points = 75
    random_coords = generate_random_coords(obstacle_positions, (x_min, x_max), (y_min, y_max), n_points)
    goal = inverse_transform_coords([goal_point], 0.05)[0]
    origin = inverse_transform_coords([origin_point], 0.05)[0]
    random_coords.extend([origin, goal])
    random_coords = np.array(random_coords)

    # Apply transformation
    s = 0.05
    transformed_random_coords = transform_coords(random_coords, s)
    transformed_obstacle_coords = transform_coords(obstacle_coords, s)

    # Build adjacency matrix
    adjacency_matrix = np.zeros((len(transformed_random_coords), len(transformed_random_coords)))
    for i in range(len(transformed_random_coords)):
        for j in range(i + 1, len(transformed_random_coords)):
            if line_of_sight(random_coords[i], random_coords[j], obstacle_positions):
                distance = get_distance(transformed_random_coords[i], transformed_random_coords[j])
                adjacency_matrix[i][j] = distance
                adjacency_matrix[j][i] = distance

    # Find the shortest path using Dijkstra's algorithm
    unvisited_nodes = list(range(len(transformed_random_coords)))
    distances = np.full(len(transformed_random_coords), np.inf)
    previous_nodes = np.full(len(transformed_random_coords), -1)
    origin_index = len(transformed_random_coords) - 2
    goal_index = len(transformed_random_coords) - 1
    distances[origin_index] = 0

    while unvisited_nodes:
        current_node = unvisited_nodes[np.argmin(distances[unvisited_nodes])]
        if distances[current_node] == np.inf or current_node == goal_index:
            break
        for neighbor, distance in enumerate(adjacency_matrix[current_node]):
            if distance and neighbor in unvisited_nodes:
                alt_distance = distances[current_node] + distance
                if alt_distance < distances[neighbor]:
                    distances[neighbor] = alt_distance
                    previous_nodes[neighbor] = current_node
        unvisited_nodes.remove(current_node)

    # Retrieve the shortest path
    shortest_path = []
    node = goal_index
    while node != -1:
        shortest_path.append(node)
        node = previous_nodes[node]
    shortest_path.reverse()
    path = [transformed_random_coords[i] for i in shortest_path]

    # Plot the results
    plt.figure(figsize=(10, 10))
    plt.scatter(transformed_obstacle_coords[:, 0], transformed_obstacle_coords[:, 1], c='b')
    plt.scatter(transformed_random_coords[:, 0], transformed_random_coords[:, 1], c='r')
    for i in range(len(transformed_random_coords)):
        for j in range(i + 1, len(transformed_random_coords)):
            if line_of_sight(random_coords[i], random_coords[j], obstacle_positions):
                plt.plot([transformed_random_coords[i, 0], transformed_random_coords[j, 0]], [transformed_random_coords[i, 1], transformed_random_coords[j, 1]], 'g-')
    for i in range(len(shortest_path) - 1):
        plt.plot([transformed_random_coords[shortest_path[i], 0], transformed_random_coords[shortest_path[i+1], 0]],
                 [transformed_random_coords[shortest_path[i], 1], transformed_random_coords[shortest_path[i+1], 1]], 'm-', linewidth=2.0)
    plt.savefig('shortest_path.png')
    print(path)
    
    return path
