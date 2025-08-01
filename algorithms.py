from geopy.distance import geodesic
import heapq
import networkx as nx

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