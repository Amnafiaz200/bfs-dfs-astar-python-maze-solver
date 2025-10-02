**# 🧩 Maze Solver in Python (BFS, DFS, A*)**

This project solves a maze using three classical search algorithms:
- **Breadth-First Search (BFS)**
- **Depth-First Search (DFS)**
- **A\* Search (with Euclidean distance heuristic)**
The program loads a maze from a CSV file, runs the chosen algorithm, and visualises both the **explored nodes** and the **final path**.

**## 🚀 Features**
- Maze loaded from a CSV configuration file
- Pathfinding using BFS, DFS, and A*
- Visualization of:
  - Explored nodes (search space)
  - Final path from start to goal
- Statistics:
  - Execution time
  - Path length
  - Number of nodes explored

**## 📂 Project Structure**
maze-solver-python/
│── main.py # Entry point, run BFS/DFS/A*
│── maze_visual.py # Maze and Agent visualization library
│── maze_config.csv # Maze configuration file
│── README.md # Documentation

**## 🛠️ Requirements**
- Python 3.x  
- Libraries:
  - `argparse` (built-in)
  - `collections` (built-in)
  - `heapq` (built-in)
  - `math` (built-in)
  - `time` (built-in)
  - `maze_visual` (provided file/module in this project)

Install any external dependencies (if required by `maze_visual`) with:
```bash
pip install -r requirements.txt

**▶️ How to Run**

Clone the repository:
git clone https://github.com/your-username/maze-solver-python.git
cd maze-solver-python
Make sure the maze configuration file is available (e.g. maze_config.csv).

Run with the algorithm of your choice:

Breadth-First Search (BFS):
python main.py --bfs

Depth-First Search (DFS):
python main.py --dfs

A Search:*
python main.py --astar

**📊 Example Output**
Running Breadth-First Search (BFS)...

--- Search Algorithm Statistics ---
Algorithm: Breadth-First Search (BFS)
Execution Time: 0.0123 seconds
Path Length: 38 steps
Nodes Explored: 152 nodes
-------------------------------------
