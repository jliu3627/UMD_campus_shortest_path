import time
import tracemalloc
import random
import math
import argparse
import networkx as nx
import heapq
from geopy.distance import geodesic
from algorithms import reconstruct_path
import osmnx as ox

# Algorithms (copied and modified from algorithms.py for benchmarking with metrics)
def haversine_distance(G: nx.MultiGraph, node1: int, node2: int):
    coord1 = (G.nodes[node1]['y'], G.nodes[node1]['x'])
    coord2 = (G.nodes[node2]['y'], G.nodes[node2]['x'])
    return geodesic(coord1, coord2).meters

def greedy_benchmark(G: nx.MultiGraph, source: int, target: int):
    # (distance_from_source, current node, path_to_target)
    visited = set()
    heap = [(haversine_distance(G, source, target), source, [source])]
    expanded = 0
    peak_frontier = len(heap)

    while heap:
        peak_frontier = max(peak_frontier, len(heap))
        _, current, path = heapq.heappop(heap)
        if current == target:
            return path, expanded, peak_frontier
        if current in visited:
            continue
        visited.add(current)
        expanded += 1
        for neighbor in G.neighbors(current):
            if neighbor not in visited:
                heapq.heappush(heap, (haversine_distance(G, neighbor, target), neighbor, path + [neighbor]))
    return None, expanded, peak_frontier

def dijkstra_benchmark(G: nx.MultiGraph, source: int, target: int):
    # (distance_from_source, current node, path_to_target)
    heap = [(0, source, [source])]
    visited = set()
    distances = {source: 0}
    expanded = 0
    peak_frontier = len(heap)

    while heap:
        peak_frontier = max(peak_frontier, len(heap))
        curr_dist, curr_node, path = heapq.heappop(heap)
        if curr_node in visited:
            continue
        visited.add(curr_node)
        expanded += 1
        if curr_node == target:
            return path, expanded, peak_frontier
        for neighbor in G.neighbors(curr_node):
            edge_weight = G.edges[curr_node, neighbor, 0].get('length', 1)
            new_dist = curr_dist + edge_weight
            if neighbor not in distances or new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                heapq.heappush(heap, (new_dist, neighbor, path + [neighbor]))
    return None, expanded, peak_frontier

def astar_benchmark(G: nx.MultiGraph, source: int, target: int):
    # (f_score, g_score, current_node, path)
    heap = [(haversine_distance(G, source, target), 0, source, [source])]
    visited = set()
    g_scores = {source: 0}
    expanded = 0
    peak_frontier = len(heap)

    while heap:
        peak_frontier = max(peak_frontier, len(heap))
        f, g, current, path = heapq.heappop(heap)
        if current in visited:
            continue
        visited.add(current)
        expanded += 1
        if current == target:
            return path, expanded, peak_frontier
        for neighbor in G.neighbors(current):
            edge_weight = G.edges[current, neighbor, 0].get('length', 1)
            tentative_g = g + edge_weight
            if tentative_g < g_scores.get(neighbor, float("inf")):
                g_scores[neighbor] = tentative_g
                f_score = tentative_g + haversine_distance(G, neighbor, target)
                heapq.heappush(heap, (f_score, tentative_g, neighbor, path + [neighbor]))
    return None, expanded, peak_frontier

def bidirectional_benchmark(G: nx.MultiGraph, source: int, target: int):
    if source == target:
        return [source], 0, 1
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
    expanded = 0
    peak_frontier = max(len(priority_start), len(priority_goal))
    best_path_nodes = None
    best_total_distance = float("inf")
    while priority_start and priority_goal:
        peak_frontier = max(peak_frontier, len(priority_start) + len(priority_goal))
        #start from start side
        if priority_start:
            distance_so_far, current_node = heapq.heappop(priority_start)
            if current_node not in visited_start:
                visited_start.add(current_node)
                expanded += 1
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
        if priority_goal:
            distance_so_far, current_node = heapq.heappop(priority_goal)
            if current_node not in visited_goal:
                visited_goal.add(current_node)
                expanded += 1
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
    return best_path_nodes, expanded, peak_frontier


def k_shortest_paths(G: nx.MultiGraph, source: int, target: int, k: int = 3):
    # First shortest path
    first_path = dijkstra_benchmark(G, source, target)[0]
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
            branch_node_path = dijkstra_benchmark(G_copy, branch_node, target)[0]
            if branch_node_path:
                total_path = root_path[:-1] + branch_node_path
                if total_path not in candidates:
                    candidates.append(total_path)
        if not candidates:
            break
        candidates.sort(key=lambda p: path_length(G, p))
        paths.append(candidates.pop(0))
    return paths

def k_shortest_paths_benchmark(G, source, target, k=3):
    paths = k_shortest_paths(G, source, target, k=k)
    return paths

def path_length(G: nx.MultiGraph, path):
    return sum(G.edges[path[i], path[i+1], 0].get("length", 1) for i in range(len(path)-1))


# Benchmark wrappers
def measure(run_fn, *args, **kwargs):
    tracemalloc.start()
    start_time = time.perf_counter()
    # The benchmark should return: (path, expanded, peak_frontier)
    path, expanded, frontier = run_fn(*args, **kwargs)
    time_ms = (time.perf_counter() - start_time) * 1000.0
    mem_current, mem_peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return path, {
        "runtime_ms": time_ms,
        # bytes to KB
        "peak_memory": mem_peak / 1024.0,   
        "nodes_expanded": expanded,
        "peak_frontier": frontier,
        "path_length": path_total_length(args[0], path)
    }

# building the graph
def load_osmnx_graph(place):
    try:

        graph_raw = ox.graph_from_place(place, network_type="walk", simplify=True)
        graph_raw = graph_raw.to_undirected()
        G = nx.MultiGraph()
        G.add_nodes_from(graph_raw.nodes(data=True))
        for u, v, data in graph_raw.edges(data=True):
            G.add_edge(u, v, length=float(data.get("length", 1.0)))
        
        # Keep largest connected part
        if not nx.is_connected(G):
            biggest = max(nx.connected_components(G), key=len)
            G = G.subgraph(biggest).copy()
        return G
    except Exception as e:
        print("OSMnx failed for", place, str(e))
        return None

def make_grid(n):
    base_lat = 38.9900
    base_lon = -76.9400
    step_lat = 0.0009
    step_lon = step_lat / math.cos(math.radians(base_lat))
    G0 = nx.grid_2d_graph(n, n)

    #Relable nodes using lat/lon coordinates
    mapping = {rc: i for i, rc in enumerate(G0.nodes())}
    G0 = nx.relabel_nodes(G0, mapping)
    for node in G0.nodes():
        r, c = divmod(node, n)
        G0.nodes[node]["y"] = base_lat + r * step_lat
        G0.nodes[node]["x"] = base_lon + c * step_lon
    for u, v in G0.edges():
        G0.edges[u, v]["length"] = 1.0
    G = nx.MultiGraph()
    # Copy nodes and edges to MultiGraph
    G.add_nodes_from(G0.nodes(data=True))
    for u, v, data in G0.edges(data=True):
        G.add_edge(u, v, **data)
    return G

def largest_cc(G):
    #Return the largest connected component of the graph
    if nx.is_connected(G):
        return G
    parts = list(nx.connected_components(G))
    big = max(parts, key=len)
    return G.subgraph(big).copy()

def sample_pairs(G, k):
    #Sample k random pairs of nodes from the graph
    nodes = list(G.nodes())
    random.shuffle(nodes)
    out = []
    tries = 0
    while len(out) < k and tries < 20 * k and len(nodes) >= 2:
        a, b = random.sample(nodes, 2)
        out.append((a, b))
        tries += 1
    return out

def path_total_length(G, path):
    #Sums the length of a full node path
    if not path or len(path) < 2:
        return None
    length = 0
    for i in range(len(path) - 1):
        length += G.edges[path[i], path[i + 1], 0].get("length", 1)
    return length

def avg(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return 0.0
    return sum(vals) / len(vals)

def profile_map(G, label, pairs, trials, k_val=3):
    # Benchmarking various algorithms on the graph G and print results
    #Algorithms to benchmark
    algos = {
        "Greedy": greedy_benchmark,
        "A*": astar_benchmark,
        "Dijkstra": dijkstra_benchmark,
        "Bidirectional": bidirectional_benchmark,
    }
    # Initialize stats dictionary
    stats = {}
    for name in algos:
        stats[name] = {
            "time": [],
            "memory": [],
            "expanded": [],
            "frontier": [],
            "path_length": [],
            "accuracy": [],
            "success": 0
        }
    
    final_pairs = sample_pairs(G, pairs)
    if not final_pairs:
        print("No pairs for: ", label)
        return
    
    # Run the algorithms for each pair
    for s, t in final_pairs:
        for i in range(trials):
            results = {}
            for name in algos:
                p, m = measure(algos[name], G, s, t)
                results[name] = {"path": p}
                results[name].update(m)
            optimal_len = results["Dijkstra"]["path_length"]
            if optimal_len is None or optimal_len <= 0:
                optimal_len = None
            for name in algos:
                info = results[name]
                if info["path"] is not None:
                    stats[name]["success"] += 1
                stats[name]["time"].append(info["runtime_ms"])
                stats[name]["memory"].append(info["peak_memory"])
                stats[name]["expanded"].append(info["nodes_expanded"])
                stats[name]["frontier"].append(info["peak_frontier"])
                stats[name]["path_length"].append(info["path_length"])
                if optimal_len is not None and info["path_length"] not in (None, 0):
                    acc = optimal_len / info["path_length"]
                    if acc > 1.0:
                        acc = 1.0
                else:
                    acc = 0.0
                stats[name]["accuracy"].append(acc)
            tracemalloc.start()
            start = time.perf_counter()
            k_paths = k_shortest_paths_benchmark(G, s, t, k=k_val)
            k_ms = (time.perf_counter() - start) * 1000
            mem1, k_peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()
        
    #print results
    print(f"\n=== {label} ===")
    print(f"Nodes: {G.number_of_nodes()} Edges: {G.number_of_edges()} "f" Pairs: {pairs} Trials/Pair: {trials}")

    order = ["Greedy", "A*", "Dijkstra", "Bidirectional"]
    total_samples = float(len(final_pairs) * trials)

    for name in order:
        time_result    = round(avg(stats[name]["time"]), 2)
        memory         = round(avg(stats[name]["memory"]), 1)
        expanded_nodes = round(avg(stats[name]["expanded"]), 1)
        frontier       = round(avg(stats[name]["frontier"]), 1)
        accuracy       = round(avg(stats[name]["accuracy"]), 3)
        success        = round(100.0 * stats[name]["success"] / total_samples, 1) if total_samples else 0.0

        print(f"\n{name}")
        print(f"  Time(ms):    {time_result:.2f}")
        print(f"  Memory(KB):     {memory:.1f}")
        print(f"  Expanded:    {expanded_nodes:.1f}")
        print(f"  Frontier:    {frontier:.1f}")
        print(f"  Accuracy:    {accuracy:.3f}")
        print(f"  Success:  {success:.1f}")


def main():
    campus = "University of Maryland, College Park"
    city = "College Park, Maryland, USA"
    pairs = 10
    trials = 1
    k = 3

    G_campus = load_osmnx_graph(campus)
    G_city = load_osmnx_graph(city)

    if G_campus is None or G_city is None:
        print("\nFailed to load OSMnx graphs so using grids instead")
        G_campus = make_grid(40)
        G_city = make_grid(120)

    profile_map(largest_cc(G_campus), f"Campus: {campus}", pairs, trials, k_val=k)
    profile_map(largest_cc(G_city),   f"Small city: {city}", pairs, trials, k_val=k)

if __name__ == "__main__":
    main()
