import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import random
import osmnx as ox
import networkx as nx
from algorithms import greedy, dijkstra, astar, k_shortest_paths, bidirectional_dijkstra
import folium
from fastapi import FastAPI, Query
from fastapi.responses import HTMLResponse

# app
app = FastAPI()

# college park map
G = ox.graph_from_place("College Park, Maryland, USA", network_type="walk", simplify=True)
G = G.to_undirected()

@app.get("/", response_class=HTMLResponse)
async def get_map():
    with open("index.html") as f:
        return HTMLResponse(f.read())
    
@app.get("/route")
async def get_route(start_lat: float, start_lon: float, end_lat: float, end_lon: float):
    source = ox.distance.nearest_nodes(G, start_lon, start_lat)
    target = ox.distance.nearest_nodes(G, end_lon, end_lat)

    # calculate shortest paths
    path_dijkstra = dijkstra(G, source, target)
    path_greedy = greedy(G, source, target)
    path_astar = astar(G, source, target)
    path_bidirectional = bidirectional_dijkstra(G, source, target)
    k_paths = k_shortest_paths(G, source, target, k=3)
    path_k_shortest = k_paths[0] if k_paths else None

    def path_to_coords(path):
        return [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in path]

    return {
        "dijkstra": path_to_coords(path_dijkstra),
        "greedy": path_to_coords(path_greedy),
        "astar": path_to_coords(path_astar),
        "bidirectional_dijkstra": path_to_coords(path_bidirectional),
        "k_shortest_path": path_to_coords(path_k_shortest) if path_k_shortest else None
    }