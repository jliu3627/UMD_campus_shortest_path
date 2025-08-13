from geopy.distance import geodesic
import heapq
import networkx as nx
from collections import deque

# Greedy
def greedy(G: nx.MultiGraph, source: int, target: int):
    visited = set()
    # (distance_from_source, current node, path_to_target)
    heap = [(haversine_distance(G, source, target), source, [source])]

    while heap:
        _, current, path = heapq.heappop(heap)
        if current == target:
            return path
        if current in visited:
            continue
        visited.add(current)

        for neighbor in G.neighbors(current):
            if neighbor not in visited:
                heapq.heappush(heap, (haversine_distance(G, neighbor, target), neighbor, path + [neighbor]))
    return None

def haversine_distance(G: nx.MultiGraph, node1: int, node2: int):
    coord1 = (G.nodes[node1]['y'], G.nodes[node1]['x'])
    coord2 = (G.nodes[node2]['y'], G.nodes[node2]['x'])
    return geodesic(coord1, coord2).meters

# Dijkstra's algorithm
def dijkstra(G: nx.MultiGraph, source: int, target: int):
    # (distance_from_source, current node, path_to_target)
    heap = [(0, source, [source])]
    visited = set()
    distances = {source: 0}

    while heap:
        curr_dist, curr_node, path = heapq.heappop(heap)
        
        if curr_node in visited:
            continue
        visited.add(curr_node)

        if curr_node == target:
            return path

        for neighbor in G.neighbors(curr_node):
            edge_weight = G.edges[curr_node, neighbor, 0].get('length', 1)
            new_dist = curr_dist + edge_weight

            if neighbor not in distances or new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                heapq.heappush(heap, (new_dist, neighbor, path + [neighbor]))
    
    return None

# astar
def astar(G: nx.MultiGraph, source: int, target: int):
    # (f_score, g_score, current_node, path)
    heap = [(haversine_distance(G, source, target), 0, source, [source])]
    visited = set()
    g_scores = {source: 0}

    while heap:
        f, g, current, path = heapq.heappop(heap)

        if current in visited:
            continue
        visited.add(current)

        if current == target:
            return path

        for neighbor in G.neighbors(current):
            edge_weight = G.edges[current, neighbor, 0].get('length', 1)
            tentative_g = g + edge_weight
            if tentative_g < g_scores.get(neighbor, float("inf")):
                g_scores[neighbor] = tentative_g
                f_score = tentative_g + haversine_distance(G, neighbor, target)
                heapq.heappush(heap, (f_score, tentative_g, neighbor, path + [neighbor]))

    return None

def reconstruct_path(pred_fwd, pred_bwd, meet_node):
    path_fwd = []
    node = meet_node
    while node is not None:
        path_fwd.append(node)
        node = pred_fwd[node]
    path_fwd.reverse()
    path_bwd = []
    node = pred_bwd[meet_node]
    while node is not None:
        path_bwd.append(node)
        node = pred_bwd[node]
    return path_fwd + path_bwd

def bidirectional_dijkstra(G: nx.MultiGraph, source: int, target: int):
    if source == target:
        return [source]

    # distances and parents for both directions
    start_dist = {source: 0}
    goal_dist  = {target: 0}
    parent_start = {source: None}
    parent_goal  = {target: None}

    # priority queues
    priority_start = [(0, source)]
    priority_goal  = [(0, target)]

    # visited sets
    visited_start = set()
    visited_goal  = set()

    best_path_nodes = None
    best_total_distance = float("inf")

    while priority_start and priority_goal:
        #start from start side 
        if priority_start:
            distance_so_far, current_node = heapq.heappop(priority_start)
            if current_node not in visited_start:
                visited_start.add(current_node)

                if current_node in visited_goal:
                    total_distance = start_dist[current_node] + goal_dist[current_node]
                    if total_distance < best_total_distance:
                        best_total_distance = total_distance
                        best_path_nodes = reconstruct_path(parent_start, parent_goal, current_node)

                for neighbor in G.neighbors(current_node):
                    edge_length = G.edges[current_node, neighbor, 0].get("length", 1)
                    new_distance = distance_so_far + edge_length
                    if new_distance < start_dist.get(neighbor, float("inf")):
                        start_dist[neighbor] = new_distance
                        parent_start[neighbor] = current_node
                        heapq.heappush(priority_start, (new_distance, neighbor))

        # start from goal side
        if priority_start:
            distance_so_far, current_node = heapq.heappop(priority_goal)
            if current_node not in visited_goal:
                visited_goal.add(current_node)

                if current_node in visited_start:
                    total_distance = start_dist[current_node] + goal_dist[current_node]
                    if total_distance < best_total_distance:
                        best_total_distance = total_distance
                        best_path_nodes = reconstruct_path(parent_start, parent_goal, current_node)

                for neighbor in G.neighbors(current_node):
                    edge_length = G.edges[current_node, neighbor, 0].get("length", 1)
                    new_distance = distance_so_far + edge_length
                    if new_distance < goal_dist.get(neighbor, float("inf")):
                        goal_dist[neighbor] = new_distance
                        parent_goal[neighbor] = current_node
                        heapq.heappush(priority_goal, (new_distance, neighbor))

    return best_path_nodes

def k_shortest_paths(G: nx.MultiGraph, source: int, target: int, k: int = 3):
    # First shortest path
    first_path = dijkstra(G, source, target)
    if not first_path:
        return []
    paths = [first_path]
    candidates = []

    for _ in range(1, k):
        # Last found shortest path
        last_path = paths[-1]
        for i in range(len(last_path) - 1):
            branch_node = last_path[i]
            root_path = last_path[:i+1]

            # Copy the graph
            G_copy = G.copy()

            for p in paths:
                if len(p) > i and p[:i+1] == root_path:
                    if G_copy.has_edge(p[i], p[i+1]):
                        G_copy.remove_edge(p[i], p[i+1])

            # Remove root path nodes
            for node in root_path[:-1]:
                if node in G_copy:
                    G_copy.remove_node(node)

            # Run Dijkstra from branch_node to target
            branch_node = dijkstra(G_copy, branch_node, target)
            if branch_node:
                total_path = root_path[:-1] + branch_node
                if total_path not in candidates:
                    candidates.append(total_path)

        if not candidates:
            break

        # Pick the best path based on total path length
        candidates.sort(key=lambda p: path_length(G, p))
        paths.append(candidates.pop(0))

    return paths

def path_length(G: nx.MultiGraph, path):
    #Calculate the total edge length of a path using Dijisktra's to use in k-shortest paths
    return sum(G.edges[path[i], path[i+1], 0].get("length", 1) for i in range(len(path)-1))
