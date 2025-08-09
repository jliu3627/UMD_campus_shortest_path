import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import random
import osmnx as ox
import networkx as nx
from algorithms import greedy, dijkstra, astar

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
    path_astar = astar(G, source, target)

    fig, ax = ox.plot_graph_routes(G, [path_dijkstra, path_greedy, path_astar],
                                route_colors=["green", "red", "blue"],
                                route_linewidth=3, node_size=0,
                                bgcolor="white", show=False)

    legend_handles = [
        mpatches.Patch(color="green", label="Dijkstra"),
        mpatches.Patch(color="red", label="Greedy Best-First"),
        mpatches.Patch(color="blue", label="A-Star")
    ]

    ax.legend(handles=legend_handles, loc="lower left")
    fig.savefig("paths_plot.png", dpi=300)