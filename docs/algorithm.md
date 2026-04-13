# Algorithm Documentation

Detailed technical documentation of the A* implementation in this project.

---

## Class: `CustomAStarSolver`

Located in `solver.py`. Encapsulates all logic for solving a grid-based maze using the A* algorithm.

### Constructor

```python
CustomAStarSolver(maze, start, end)
```

| Parameter | Type | Description |
|---|---|---|
| `maze` | `list[list[int]]` | 2D grid where `0` = open, `1` = wall |
| `start` | `tuple(int, int)` | Starting `(row, col)` coordinate |
| `end` | `tuple(int, int)` | Goal `(row, col)` coordinate |

---

## Methods

### `custom_heuristic(position)`

Modified Manhattan distance. Applies a square root to the sum of absolute coordinate differences.

```python
def custom_heuristic(self, position):
    return (abs(position[0] - self.end[0]) + abs(position[1] - self.end[1])) ** 0.5
```

**Why the square root?**
Standard Manhattan distance is `|dx| + |dy|`. Applying `** 0.5` produces a value smaller than the true cost — the heuristic becomes *more conservative* (closer to Dijkstra's). This is useful in mazes where the straight-line estimate would be too aggressive and skip good paths.

---

### `get_directions()`

Returns the four cardinal movement directions.

```python
def get_directions(self):
    return [(-1, 0), (1, 0), (0, -1), (0, 1)]
```

Movement is restricted to up, down, left, right — no diagonals.

---

### `is_valid_position(position)`

Checks that a position is within maze bounds and not a wall.

```python
def is_valid_position(self, position):
    return (
        0 <= position[0] < len(self.maze)
        and 0 <= position[1] < len(self.maze[0])
        and self.maze[position[0]][position[1]] == 0
    )
```

---

### `find_neighbors(position)`

Returns all valid neighbors of a given position.

```python
def find_neighbors(self, position):
    directions = self.get_directions()
    neighbors = []
    for dx, dy in directions:
        new_position = (position[0] + dx, position[1] + dy)
        if self.is_valid_position(new_position):
            neighbors.append(new_position)
    return neighbors
```

---

### `step_cost(current, neighbor)`

Returns the movement cost between adjacent cells. Fixed at `1` for uniform grids.

```python
def step_cost(self, current, neighbor):
    return 1
```

---

### `reconstruct_path(came_from)`

Traces back through `came_from` from `end` to `start`, then reverses the result.

```python
def reconstruct_path(self, came_from):
    path = []
    current = self.end
    while current in came_from:
        path.append(current)
        current = came_from[current]
    return path[::-1]
```

---

### `solve()`

Main A* loop. Returns the optimal path or an empty list if none exists.

```python
def solve(self):
    open_set = []
    heapq.heappush(open_set, (0, self.start))
    came_from = {}
    g_score = {self.start: 0}
    f_score = {self.start: self.custom_heuristic(self.start)}

    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == self.end:
            return self.reconstruct_path(came_from)
        for neighbor in self.find_neighbors(current):
            tentative_g_score = g_score[current] + self.step_cost(current, neighbor)
            if tentative_g_score < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + self.custom_heuristic(neighbor)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []
```

**Step-by-step:**

1. Initialize the open set with the start node at cost 0
2. Pop the node with the lowest `f(n)` from the heap
3. If it's the goal — reconstruct and return the path
4. For each valid neighbor, compute `tentative_g_score`
5. If this is a better path to the neighbor — update scores and push to heap
6. Repeat until the goal is reached or the open set is empty

---

## Heuristic Comparison

| Heuristic | Formula | Best for |
|---|---|---|
| Manhattan distance | `\|dx\| + \|dy\|` | Grid movement without diagonals |
| Euclidean distance | `sqrt(dx² + dy²)` | Grid movement with diagonals allowed |
| Modified Manhattan (this project) | `sqrt(\|dx\| + \|dy\|)` | Conservative path exploration in dense mazes |

---

## Time and Space Complexity

| | Complexity |
|---|---|
| Time | O(E log V) where E = edges, V = vertices (cells) |
| Space | O(V) for open set, came_from, g_score, f_score |

In the worst case (open maze, no walls), all cells are visited. In practice, the heuristic significantly reduces the number of explored nodes.
