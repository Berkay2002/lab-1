import heapq
import time
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# -------------------- HEURISTICS --------------------

def h1(state, goal):
    return sum(1 for i in range(9) if state[i] != goal[i] and state[i] != 0)

def h2(state, goal):
    distance = 0
    for i in range(1, 9):
        xi, yi = divmod(state.index(i), 3)
        xg, yg = divmod(goal.index(i), 3)
        distance += abs(xi - xg) + abs(yi - yg)
    return distance

# -------------------- A* ALGORITHM --------------------

def get_neighbors(state):
    neighbors = []
    idx = state.index(0)
    row, col = divmod(idx, 3)
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for dr, dc in moves: 
        r, c = row + dr, col + dc
        if 0 <= r < 3 and 0 <= c < 3:
            new_idx = r * 3 + c
            new_state = list(state)
            new_state[idx], new_state[new_idx] = new_state[new_idx], new_state[idx]
            neighbors.append((tuple(new_state), ""))
    return neighbors

def a_star(start, goal, heuristic):
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), 0, start, []))
    visited = set() # Hashes the state (Constant time on average)

    while frontier:
        f, g, state, path = heapq.heappop(frontier)
        if state == goal:
            return path + [state]
        
        if state in visited:
            continue

        visited.add(state)
        for new_state, _ in get_neighbors(state):
            if new_state not in visited:
                h = heuristic(new_state, goal)
                heapq.heappush(frontier, (g + 1 + h, g + 1, new_state, path + [state]))
    return None

# -------------------- ANIMATION --------------------

def get_tile_positions(state):
    pos = {}
    for i, val in enumerate(state):
        if val == 0: continue
        row, col = divmod(i, 3)
        pos[val] = (col, 2 - row)
    return pos

def interpolate_positions(p1, p2, alpha):
    return {
        k: (
            p1[k][0] * (1 - alpha) + p2[k][0] * alpha,
            p1[k][1] * (1 - alpha) + p2[k][1] * alpha
        ) for k in p1
    }

def make_positions(solution):
    return [get_tile_positions(state) for state in solution]

def animate(frame):
    ax.clear()
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(-0.5, 2.5); ax.set_ylim(-0.5, 2.5)
    ax.set_aspect('equal')
    ax.set_facecolor('#111')

    if frame < total_frames:
        i = frame // smooth_frames
        alpha = (frame % smooth_frames) / smooth_frames
        i = min(i, len(positions) - 2)
        interp = interpolate_positions(positions[i], positions[i + 1], alpha)
    else:
        interp = positions[-1]

    for tile, (x, y) in interp.items():
        rect = plt.Rectangle((x - 0.45, y - 0.45), 0.9, 0.9,
                             fc='#00cc88', ec='black', linewidth=2)
        ax.add_patch(rect)
        ax.text(x, y, str(tile), ha='center', va='center',
                fontsize=24, weight='bold', color='white')

# -------------------- RANDOM START STATE --------------------

def is_solvable(state):
    inv_count = 0
    tiles = [tile for tile in state if tile != 0]
    for i in range(len(tiles)):
        for j in range(i + 1, len(tiles)):
            if tiles[i] > tiles[j]:
                inv_count += 1
    return inv_count % 2 == 0

def generate_random_start(goal):
    while True:
        state = list(goal)
        random.shuffle(state)
        if tuple(state) != goal and is_solvable(state):
            return tuple(state)

# -------------------- RUN --------------------

goal = (1, 2, 3,
        4, 5, 6,
        7, 8, 0)

start = generate_random_start(goal)
print("Start State:")
for i in range(0, 9, 3):
    print(start[i:i+3])
print()

# Solve with h1
start_time_h1 = time.time()
solution_h1 = a_star(start, goal, h1)
end_time_h1 = time.time()

# Solve with h2
start_time_h2 = time.time()
solution_h2 = a_star(start, goal, h2)
end_time_h2 = time.time()

# Console output
steps_h1 = len(solution_h1) - 1
steps_h2 = len(solution_h2) - 1
time_h1 = end_time_h1 - start_time_h1
time_h2 = end_time_h2 - start_time_h2

print(f"h1 (Misplaced Tiles): {steps_h1} steps, {time_h1:.4f} sec")
print(f"h2 (Manhattan Distance): {steps_h2} steps, {time_h2:.4f} sec")

if steps_h1 < steps_h2:
    print("✅ h1 found a shorter solution.")
elif steps_h2 < steps_h1:
    print("✅ h2 found a shorter solution.")
else:
    print("⚖️  Both heuristics found solutions with the same number of steps.")

if time_h1 < time_h2:
    print("⚡ h1 was faster in time.")
elif time_h2 < time_h1:
    print("⚡ h2 was faster in time.")
else:
    print("⏱️  Both ran in roughly the same time.")

# Pick which to animate:
animate_heuristic = "h2"  # or "h1"
solution = solution_h1 if animate_heuristic == "h1" else solution_h2

# Animation settings
smooth_frames = 10
interval_ms = 50
positions = make_positions(solution)
total_frames = len(positions) * smooth_frames

# Launch animation
fig, ax = plt.subplots(figsize=(4, 4))
ani = animation.FuncAnimation(fig, animate,
                              frames=total_frames,
                              interval=interval_ms, repeat=False)
plt.show()
