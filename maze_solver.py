"""
Syntecxhub AI Week 1 - Project 1
Maze Solver using A* Search

Legend:
S = Start, G = Goal, # = Wall, . = Open path
"""

import heapq

MAZE = [
    ["S", ".", ".", "#", "."],
    ["#", "#", ".", "#", "."],
    [".", ".", ".", ".", "."],
    [".", "#", "#", "#", "."],
    [".", ".", ".", ".", "G"],
]


def find_marker(maze, marker):
    for row, cells in enumerate(maze):
        for col, cell in enumerate(cells):
            if cell == marker:
                return (row, col)
    raise ValueError(f"Maze is missing marker {marker!r}")


def heuristic(a, b):
    """Manhattan distance: suitable for four-direction movement."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(position, maze):
    row, col = position
    rows, cols = len(maze), len(maze[0])
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr, nc = row + dr, col + dc
        if 0 <= nr < rows and 0 <= nc < cols and maze[nr][nc] != "#":
            yield (nr, nc)


def astar(maze):
    start = find_marker(maze, "S")
    goal = find_marker(maze, "G")

    # Priority queue entries: (estimated_total_cost, cost_so_far, position)
    frontier = [(heuristic(start, goal), 0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return list(reversed(path))

        # Ignore outdated queue entries
        if current_cost != cost_so_far[current]:
            continue

        for neighbor in get_neighbors(current, maze):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, new_cost, neighbor))

    return None  # Goal cannot be reached


def show_maze(maze, path=None):
    display = [row[:] for row in maze]
    if path:
        for row, col in path:
            if display[row][col] not in ("S", "G"):
                display[row][col] = "*"
    for row in display:
        print(" ".join(row))


if __name__ == "__main__":
    print("Original maze:")
    show_maze(MAZE)
    path = astar(MAZE)
    if path is None:
        print("\\nNo path found: the goal is unreachable.")
    else:
        print("\\nSolved maze (* marks the shortest path):")
        show_maze(MAZE, path)
        print("\\nPath coordinates (row, column):")
        print(path)
        print("Number of moves:", len(path) - 1)
