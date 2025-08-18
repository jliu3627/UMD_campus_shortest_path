# UMD Campus Shortest Path Finder

An interactive tool for finding the shortest walking paths across the University of Maryland, College Park campus.  
The project implements and compares multiple shortest-path algorithms on real pedestrian network data using **OpenStreetMap** (via OSMnx).

---

## ✨ Features
- **Algorithms Implemented**
  - Dijkstra
  - A* Search
  - Greedy Best-First Search
  - Bidirectional Dijkstra
  - K-Shortest Paths
- **Interactive Map**
  - Select start and end points with mouse clicks
  - Visualizes routes with distinct colors for each algorithm
- **Benchmarking**
  - Performance evaluation at campus and city scales
  - Metrics: runtime, memory usage, nodes expanded, accuracy

---

## 🛠️ Installation

Clone the repository and set up the environment:

```bash
git clone https://github.com/jliu3627/UMD_campus_shortest_path.git
cd UMD_campus_shortest_path
bash setup.sh
