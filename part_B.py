import heapq  # We use a priority queue to always expand the most promising state first
import time   # For measuring how long the algorithm takes to solve the puzzle

def h2(state, goal):
    """
    Heuristic function (h2): Manhattan Distance
    For each tile, we calculate how many moves it needs (up/down/left/right)
    to reach its correct position in the goal state.
    This gives the algorithm a more detailed estimate of how close it is to the goal.
    """
    distance = 0
    for i in range(1, 9):  # We skip 0 (blank space) since it doesn't contribute
        xi, yi = divmod(state.index(i), 3)  # Current position of tile i
        xg, yg = divmod(goal.index(i), 3)   # Goal position of tile i
        distance += abs(xi - xg) + abs(yi - yg)
    return distance

def get_neighbors(state):
    """
    Generates all possible states from the current state by moving the blank tile (0)
    in all 4 directions: up, down, left, right. 
    Only valid moves within the 3x3 grid are returned.
    """
    neighbors = []
    idx = state.index(0)  # Get the index of the blank tile
    row, col = divmod(idx, 3)  # Convert index to row and column

    # Define direction vectors: (row_change, column_change)
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

    for dr, dc in moves:
        r, c = row + dr, col + dc
        if 0 <= r < 3 and 0 <= c < 3:
            new_idx = r * 3 + c
            new_state = list(state)
            # Swap blank with the neighboring tile
            new_state[idx], new_state[new_idx] = new_state[new_idx], new_state[idx]
            # Convert list back to tuple and store with a move description
            neighbors.append((tuple(new_state), f"Move tile {state[new_idx]}"))

    return neighbors

def a_star(start, goal, heuristic):
    """
    Main A* algorithm.
    We use a priority queue to pick the state with the lowest estimated total cost (f = g + h).
    g: Cost to reach current state
    h: Heuristic estimate to goal
    The algorithm stops once the goal is reached or all options are exhausted.
    """
    frontier = []  # This is our priority queue of states to explore
    heapq.heappush(frontier, (heuristic(start, goal), 0, start, []))  # Push the start node
    visited = set()  # To avoid revisiting the same state

    while frontier:
        f, g, state, path = heapq.heappop(frontier)  # Always expand the most promising node

        if state == goal:
            return path + [state]  # Found the goal — return the full path to it

        if state in visited:
            continue  # Skip already explored states
        visited.add(state)

        for new_state, move in get_neighbors(state):
            if new_state not in visited:
                h = heuristic(new_state, goal)  # Estimate cost to goal
                # Push the new state into the frontier with updated cost and path
                heapq.heappush(frontier, (g + 1 + h, g + 1, new_state, path + [state]))

    return None  # No solution was found (shouldn't happen if puzzle is solvable)

def print_board(state):
    """
    Helper function to print the puzzle nicely as a 3x3 grid.
    Converts 0 to blank space for clarity.
    """
    for i in range(0, 9, 3):
        row = [' ' if n == 0 else str(n) for n in state[i:i+3]]
        print(" ".join(row))
    print()

# -------------------------------
# Define the START and GOAL states of the puzzle
start = (7, 2, 4,
         5, 0, 6,
         8, 3, 1)  # Challenging start state

goal = (1, 2, 3,
        4, 5, 6,
        7, 8, 0)  # Standard goal state for 8-puzzle
# -------------------------------

# Start the timer to measure performance
start_time = time.time()

# Run A* search using the Manhattan Distance heuristic (h2)
solution = a_star(start, goal, h2)

# End the timer once search is complete
end_time = time.time()
elapsed = end_time - start_time

# Output results
if solution:
    print("Solution found! Steps:")
    for idx, step in enumerate(solution):
        print(f"Step {idx}:")
        print_board(step)
    print(f"Solved in {elapsed:.4f} seconds.")  # Total solve time
else:
    print("No solution found.")
    print(f"Tried for {elapsed:.4f} seconds.")
