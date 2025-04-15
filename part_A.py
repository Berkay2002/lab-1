import heapq  # To maintain a priority queue for selecting the best next state
import time   # For measuring the time taken by the algorithm

def h1(state, goal):
    """
    Heuristic function (h1): Number of misplaced tiles
    This counts how many tiles are not in their correct position,
    excluding the blank tile (0). The more misplaced tiles, the further we are.
    """
    return sum(1 for i in range(9) if state[i] != goal[i] and state[i] != 0)

def get_neighbors(state):
    """
    Generates all valid next states by moving the blank tile (0).
    The blank can move up, down, left, or right — as long as it stays in bounds.
    Each neighbor is a new puzzle configuration reachable in one move.
    """
    neighbors = []
    idx = state.index(0)  # Find the index of the blank tile
    row, col = divmod(idx, 3)  # Convert index to (row, col)

    # Define possible directions: (row_offset, col_offset)
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

    for dr, dc in moves:
        r, c = row + dr, col + dc
        if 0 <= r < 3 and 0 <= c < 3:
            new_idx = r * 3 + c
            new_state = list(state)
            # Swap the blank with the adjacent tile
            new_state[idx], new_state[new_idx] = new_state[new_idx], new_state[idx]
            neighbors.append((tuple(new_state), f"Move tile {state[new_idx]}"))

    return neighbors

def a_star(start, goal, heuristic):
    """
    - f(n) = g(n) + h(n)
    - g(n): cost to reach current state (depth)
    - h(n): estimated cost from current to goal (heuristic)
    The algorithm picks the state with the lowest total estimated cost.
    """
    frontier = []  # The priority queue for frontier states
    heapq.heappush(frontier, (heuristic(start, goal), 0, start, []))  # Push initial state
    visited = set()  # Set of already-explored states to avoid revisits

    while frontier: # Binary heap (log n time)
        f, g, state, path = heapq.heappop(frontier)  # Pops the state with the lowest f-score

        if state == goal:
            return path + [state]  # Goal reached, return full path

        if state in visited:
            continue  # Skip if we've already seen this state
        visited.add(state)

        # Explore neighbors and add them to the frontier
        for new_state, move in get_neighbors(state): # Sliding the blank tile to all possible positions
            if new_state not in visited:
                h = heuristic(new_state, goal)  # Estimate cost from new_state to goal
                heapq.heappush(frontier, (g + 1 + h, g + 1, new_state, path + [state]))

    return None  # If goal can't be reached (shouldn't happen for solvable puzzles)

def print_board(state):
    """
    Nicely prints a given puzzle state as a 3x3 grid.
    Replaces 0 with a blank for clarity.
    """
    for i in range(0, 9, 3):
        row = [' ' if n == 0 else str(n) for n in state[i:i+3]]
        print(" ".join(row))
    print()

# -------------------------------
# Define START and GOAL states
start = (8, 6, 7,
         2, 5, 4,
         3, 0, 1)  # Challenging start state

goal = (1, 2, 3,
        4, 5, 6,
        7, 8, 0)  # Classic solved puzzle layout
# -------------------------------

# Start timer to measure how long it takes to solve
start_time = time.time()

# Run A* using h1 (misplaced tiles) heuristic
solution = a_star(start, goal, h1)

# End timer and calculate duration
end_time = time.time()
elapsed = end_time - start_time

# Display the solution path step-by-step
if solution:
    print("Solution found! Steps:")
    for idx, step in enumerate(solution):
        print(f"Step {idx}:")
        print_board(step)
    print(f"Solved in {elapsed:.4f} seconds.")  # Show total time taken
else:
    print("No solution found.")
    print(f"Tried for {elapsed:.4f} seconds.")
