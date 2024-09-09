import heapq

# Define the possible moves (up, down, left, right)
moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]

# Define the heuristic function (Manhattan distance)
def heuristic(state, goal):
    return sum(abs(b - g) for a, b in enumerate(state) for g in goal if a != g)

# Define the A* search algorithm
def astar_search(start, goal):
    open_list = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            break

        for dx, dy in moves:
            x, y = current.index(0) % 3, current.index(0) // 3
            nx, ny = x + dx, y + dy

            if 0 <= nx < 3 and 0 <= ny < 3:
                neighbor = list(current)
                neighbor[x + y * 3], neighbor[nx + ny * 3] = neighbor[nx + ny * 3], neighbor[x + y * 3]

                new_cost = cost_so_far[current] + 1
                if tuple(neighbor) not in cost_so_far or new_cost < cost_so_far[tuple(neighbor)]:
                    cost_so_far[tuple(neighbor)] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (priority, tuple(neighbor)))
                    came_from[tuple(neighbor)] = current

    # Reconstruct the path
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path

# Define the start and goal states
start = (1, 2, 3, 4, 5, 6, 7, 8, 0)
goal = (1, 2, 3, 4, 5, 6, 7, 0, 8)

# Solve the puzzle
path = astar_search(start, goal)

# Print the solution
for state in path:
    print(state)
