from solver import CustomAStarSolver


def print_maze_with_path(maze, path, start, end):
    """
    Prints the maze to the console with the found path marked.

    Legend:
        S  — start position
        E  — end position
        *  — path cell
        #  — wall
        .  — open cell
    """
    path_set = set(path)
    rows = len(maze)
    cols = len(maze[0])

    print()
    for r in range(rows):
        row_str = ""
        for c in range(cols):
            pos = (r, c)
            if pos == start:
                row_str += " S "
            elif pos == end:
                row_str += " E "
            elif pos in path_set:
                row_str += " * "
            elif maze[r][c] == 1:
                row_str += " # "
            else:
                row_str += " . "
        print(row_str)
    print()


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

    if path:
        print(f"Path found ({len(path)} steps): {path}")
        print_maze_with_path(maze, path, start, end)
    else:
        print("No path found.")
