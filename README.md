# Pathfinding Agent Visualization
A Python GUI application to visualize pathfinding algorithms (A* and Greedy Best-First Search) on a customizable grid with dynamic obstacles.
## Features
A* Search (with Manhattan or Euclidean heuristic)
Greedy Best-First Search(with Manhattan or Euclidean heuristic)
## Dynamic Obstacles
Enable "Dynamic Mode" to let random obstacles appear while the agent moves, triggering automatic replanning.
## Customizable Grid
Adjust number of rows, columns, and obstacle density.
Draw walls, set start and goal positions, or erase obstacles.
## Real-Time Metrics
Nodes visited
Path cost
Algorithm execution time
## Visualization
Step-by-step animation of algorithm execution.
Agent movement along the discovered path.
## Interactive Map Editor
Draw, erase, or reposition start/goal positions.
## Maze Generation
Generate random mazes with specified obstacle density.

## Installation
1. Ensure you have **Python 3.8+** installed.
2. Install required libraries (only Tkinter is needed; it comes pre-installed with Python):
pip install tk
git clone https://github.com/yourusername/pathfinding-agent.git
cd pathfinding-agent
