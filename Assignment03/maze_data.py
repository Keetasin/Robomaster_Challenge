import heapq
from typing import Dict, List, Tuple
import pandas as pd

def dijkstra_multi_target(graph: Dict[Tuple[int, int], List[Tuple[int, int]]], start: Tuple[int, int], targets: List[Tuple[int, int]], chicks: List[Tuple[int, int]]) -> Tuple[Dict[Tuple[int, int], int], Dict[Tuple[int, int], List[Tuple[int, int]]]]:
    def single_target_dijkstra(graph, start, end):
        distances = {node: float('inf') for node in graph}
        distances[start] = 0
        pq = [(0, start)]
        visited = set()
        previous = {node: None for node in graph}

        while pq:
            current_dist, current = heapq.heappop(pq)

            if current == end:
                path = []
                while current:
                    path.append(current)
                    current = previous[current]
                return current_dist, path[::-1]

            if current in visited:
                continue

            visited.add(current)

            for neighbor in graph[current]:
                new_dist = current_dist + 1  # Assuming all edges have weight 1

                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))

        return -1, []  # No path found

    total_distances = {}
    total_paths = {}

    for target in targets:
        distance, path = single_target_dijkstra(graph, start, target)
        if distance != -1:
            total_distances[target] = distance
            total_paths[target] = path
        else:
            total_distances[target] = -1  # No path found for this target
            total_paths[target] = []  # Empty path

    return total_distances, total_paths

def convert_path_to_dict(path, maze_size):
    maze_dict = {(i, j): [] for i in range(maze_size[0]) for j in range(maze_size[1])}
    
    def add_connection(from_cell, to_cell):
        if to_cell not in maze_dict[from_cell]:
            maze_dict[from_cell].append(to_cell)
        if from_cell not in maze_dict[to_cell]:
            maze_dict[to_cell].append(from_cell)
    
    for i in range(len(path) - 1):
        current_cell = path[i]
        next_cell = path[i + 1]
        add_connection(current_cell, next_cell)
    
    maze_dict = {k: v for k, v in maze_dict.items() if v}
    
    return maze_dict

# Function to load direction and list_travel from CSV
def get_maze_data():
    maze_size = (6, 6)

    df_travel = pd.read_csv('list_path.csv')
    df_acrylic= pd.read_csv('list_acrylic.csv')
    df_chicken = pd.read_csv('list_chicken.csv')

    direction = [item for item in df_travel["direction"]] 
    traveled_path = [eval(item) for item in df_travel['travel']]

    maze = convert_path_to_dict(traveled_path, maze_size)
    start = traveled_path[0]
    targets = [eval(item) for item in df_acrylic['acrylic']]
    chicks = [eval(item) for item in df_chicken['chicken']]

    return maze_size, traveled_path, maze, start, targets, chicks


