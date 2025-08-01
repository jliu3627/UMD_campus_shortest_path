import matplotlib.pyplot as plt
import random
import osmnx as ox
import networkx as nx
from algorithms import greedy, dijkstra

if __name__ == "__main__":
    # college park map
    G = ox.graph_from_place("College Park, Maryland, USA", network_type="walk", simplify=True)
    G = G.to_undirected()

    # select two random nodes
    nodes = list(G.nodes)
    source = random.choice(nodes)
    target = random.choice(nodes)
    print(f"Source node: {source}")
    print(f"Target node: {target}")

    path_dijkstra = dijkstra(G, source, target)
    path_greedy = greedy(G, source, target)

    fig, ax = ox.plot_graph_routes(G, [path_dijkstra, path_greedy],
                                route_colors=["green", "red"],
                                route_linewidth=3, node_size=0,
                                bgcolor="white", show=False)

    ax.legend(["Dijkstra", "Greedy Best-First"], loc="lower left")
    fig.savefig("paths_plot.png", dpi=300)