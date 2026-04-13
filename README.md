# Maze Route Optimization Using A* Algorithm

A Python implementation of the A* pathfinding algorithm for solving maze navigation problems, using a modified Manhattan distance heuristic.

---

## Overview

This project implements a custom A* algorithm (`CustomAStarSolver`) that finds the optimal path through a grid-based maze from a start position to a goal position. The implementation uses a **modified Manhattan distance heuristic** (with a square root applied) which makes the algorithm slightly more aggressive in exploring promising paths.

---

## Project Structure

```
maze-astar/
│
├── solver.py          # Core A* algorithm implementation
├── visualize.py       # ASCII maze visualization with path overlay
├── requirements.txt   # Python dependencies (none beyond stdlib)
│
├── docs/
│   └── algorithm.md   # Detailed algorithm and heuristic documentation
│
└── README.md
```

---

## Technology Stack

| Tool / Library | Purpose |
|---|---|
| Python 3.x | Implementation language |
| `heapq` | Min-heap priority queue for the open set |
| `collections` | Auxiliary data structures |

No external dependencies — runs on standard Python 3.

---

## Algorithm

The A* algorithm finds the shortest path from `start` to `end` by evaluating nodes using the cost function:

```
f(n) = g(n) + h(n)
```

| Symbol | Meaning |
|---|---|
| `g(n)` | Actual cost from start to node `n` |
| `h(n)` | Estimated (heuristic) cost from `n` to goal |
| `f(n)` | Total estimated cost through node `n` |

### Custom heuristic

Instead of the standard Manhattan distance, this implementation applies a square root:

```python
def custom_heuristic(self, position):
    return (abs(position[0] - self.end[0]) + abs(position[1] - self.end[1])) ** 0.5
```

This makes the heuristic more conservative (closer to uniform-cost search) which can be beneficial in dense mazes where aggressive heuristics overshoot.

### Data structures

| Structure | Role |
|---|---|
| `open_set` (min-heap) | Nodes to be explored, ordered by `f(n)` |
| `came_from` (dict) | Tracks each node's predecessor for path reconstruction |
| `g_score` (dict) | Best known cost from start to each node |
| `f_score` (dict) | Estimated total cost through each node |

---

## Maze Format

The maze is a 2D Python list:

- `0` — open cell (passable)
- `1` — wall (blocked)

```python
maze = [
    [0, 0, 0, 0, 1],
    [1, 1, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0],
]

start = (0, 0)   # top-left
end   = (4, 4)   # bottom-right
```

---

## Setup & Usage

No installation required beyond Python 3.

### Run the solver

```bash
python solver.py
```

**Output:**
```
Path found: [(0, 1), (0, 2), (0, 3), (1, 3), (1, 4), (2, 4), (3, 4), (4, 4)]
```

### Run with ASCII visualization

```bash
python visualize.py
```

**Output:**
```
Path found (8 steps): [(0, 1), (0, 2), (0, 3), (1, 3), (1, 4), (2, 4), (3, 4), (4, 4)]

 S  *  *  *  #
 #  #  .  *  *
 .  .  #  .  *
 .  .  #  #  *
 .  .  .  .  E
```

---

## Key Concepts

| Term | Description |
|---|---|
| **A* algorithm** | Informed search algorithm that combines actual path cost with a heuristic estimate |
| **Heuristic** | A function that estimates the cost to reach the goal — must never overestimate for guaranteed optimality |
| **Manhattan distance** | Sum of absolute horizontal and vertical differences between two points — used for grid-based movement without diagonals |
| **Euclidean distance** | Straight-line distance between two points — used when diagonal movement is allowed |
| **Open set** | Priority queue of nodes yet to be explored |
| **Closed set / came_from** | Tracks explored nodes and their predecessors |
| **Admissible heuristic** | A heuristic that never overestimates the true cost — guarantees A* finds the optimal path |
| **g(n)** | Actual cost accumulated from the start node to node n |
| **h(n)** | Heuristic estimate of the cost from node n to the goal |
| **f(n)** | Total estimated cost: f(n) = g(n) + h(n) |

---

## Applications of A*

The A* algorithm is used across multiple domains:

- **Robotics** — path planning for industrial robots and autonomous vehicles
- **GPS navigation** — calculating optimal routes in real time, accounting for traffic
- **Game AI** — NPC movement through complex game maps
- **Research robotics** — navigation in unknown environments (space exploration, underwater)

---
