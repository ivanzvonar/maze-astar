import heapq


class CustomAStarSolver:
    """
    A* algorithm implementation for maze pathfinding.
    Uses a modified Manhattan distance heuristic (with square root)
    for a slightly more aggressive path exploration.
    """

    def __init__(self, maze, start, end):
        """
        Initialize the solver with a maze, start, and end position.

        Args:
            maze  : 2D list where 0 = open path, 1 = wall
            start : (row, col) tuple — starting position
            end   : (row, col) tuple — target position
        """
        self.maze = maze
        self.start = start
        self.end = end

    def custom_heuristic(self, position):
        """
        Modified Manhattan distance heuristic.
        Applies square root to the sum of absolute differences,
        making the algorithm slightly more aggressive in exploring paths.

        Args:
            position: (row, col) tuple of the current node

        Returns:
            float: estimated cost from position to end
        """
        return (abs(position[0] - self.end[0]) + abs(position[1] - self.end[1])) ** 0.5

    def get_directions(self):
        """
        Returns the four cardinal movement directions (up, down, left, right).

        Returns:
            list of (dx, dy) tuples
        """
        return [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def is_valid_position(self, position):
        """
        Checks whether a position is within maze bounds and not a wall.

        Args:
            position: (row, col) tuple

        Returns:
            bool: True if position is valid and passable
        """
        return (
            0 <= position[0] < len(self.maze)
            and 0 <= position[1] < len(self.maze[0])
            and self.maze[position[0]][position[1]] == 0
        )

    def find_neighbors(self, position):
        """
        Finds all valid neighboring positions from the current position.

        Args:
            position: (row, col) tuple

        Returns:
            list of valid (row, col) neighbor tuples
        """
        directions = self.get_directions()
        neighbors = []
        for dx, dy in directions:
            new_position = (position[0] + dx, position[1] + dy)
            if self.is_valid_position(new_position):
                neighbors.append(new_position)
        return neighbors

    def step_cost(self, current, neighbor):
        """
        Returns the cost of moving from current to a neighboring cell.
        Fixed at 1 for uniform-cost grids.

        Args:
            current  : (row, col) current position
            neighbor : (row, col) neighboring position

        Returns:
            int: movement cost (always 1)
        """
        return 1

    def reconstruct_path(self, came_from):
        """
        Reconstructs the path from end to start by following came_from links.

        Args:
            came_from: dict mapping each node to its predecessor

        Returns:
            list of (row, col) tuples from start to end
        """
        path = []
        current = self.end
        while current in came_from:
            path.append(current)
            current = came_from[current]
        return path[::-1]

    def solve(self):
        """
        Runs the A* algorithm on the maze.

        Returns:
            list of (row, col) tuples representing the optimal path,
            or an empty list if no path exists
        """
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


if __name__ == "__main__":
    maze = [
        [0, 0, 0, 0, 1],
        [1, 1, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0],
    ]

    start = (0, 0)
    end = (4, 4)

    solver = CustomAStarSolver(maze, start, end)
    path = solver.solve()

    print("Path found:", path)
