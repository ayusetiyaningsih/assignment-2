import heapq
import time

# Grid kota
grid = [
    ['S', '.', '.', 'T', '.'],
    ['T', 'T', '.', 'T', '.'],
    ['.', '.', '.', '.', '.'],
    ['.', 'T', 'T', '.', '.'],
    ['.', '.', '.', 'T', 'H']
]

ROWS, COLS = len(grid), len(grid[0])

# Posisi Start dan Goal
def find_position(symbol):
    for i in range(ROWS):
        for j in range(COLS):
            if grid[i][j] == symbol:
                return (i, j)
    return None

start = find_position('S')
goal = find_position('H')

# Heuristik Manhattan Distance
def manhattan(pos1, pos2):
    return abs(pos1[0]-pos2[0]) + abs(pos1[1]-pos2[1])

# Fungsi validasi gerak
def is_valid(pos):
    x, y = pos
    return 0 <= x < ROWS and 0 <= y < COLS and grid[x][y] != 'T'

# Gerakan (atas, bawah, kiri, kanan)
directions = [(-1,0), (1,0), (0,-1), (0,1)]

# GBFS
def gbfs(start, goal):
    queue = []
    heapq.heappush(queue, (manhattan(start, goal), start))
    came_from = {}
    visited = set()

    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            break
        visited.add(current)

        for dx, dy in directions:
            neighbor = (current[0]+dx, current[1]+dy)
            if is_valid(neighbor) and neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(queue, (manhattan(neighbor, goal), neighbor))

    path = []
    cur = goal
    while cur in came_from:
        path.append(cur)
        cur = came_from[cur]
    path.append(start)
    return path[::-1], visited

# A* opsional
def a_star(start, goal):
    queue = []
    heapq.heappush(queue, (0, start))
    came_from = {}
    g_score = {start: 0}
    visited = set()

    while queue:
        _, current = heapq.heappop(queue)
        visited.add(current)

        if current == goal:
            break

        for dx, dy in directions:
            neighbor = (current[0]+dx, current[1]+dy)
            if is_valid(neighbor):
                temp_g = g_score[current] + 1
                if neighbor not in g_score or temp_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g
                    f = temp_g + manhattan(neighbor, goal)
                    heapq.heappush(queue, (f, neighbor))

    path = []
    cur = goal
    while cur in came_from:
        path.append(cur)
        cur = came_from[cur]
    path.append(start)
    return path[::-1], visited

# Jalankan GBFS
start_time_g = time.time()
path_g, visited_g = gbfs(start, goal)
end_time_g = time.time()
time_g = (end_time_g - start_time_g) * 1000

# Jalankan A*
start_time_a = time.time()
path_a, visited_a = a_star(start, goal)
end_time_a = time.time()
time_a = (end_time_a - start_time_a) * 1000

# Tampilkan Grid dengan Path (GBFS)
def show_grid(path, label):
    print(f"\n=== Grid Path {label} ===")
    new_grid = [row.copy() for row in grid]
    for (x, y) in path:
        if new_grid[x][y] not in ('S', 'H'):
            new_grid[x][y] = '*'
    for row in new_grid:
        print(' '.join(row))

show_grid(path_g, "GBFS")
show_grid(path_a, "A*")

# Output
print("\n=== Hasil GBFS ===")
print("Rute :", path_g)
print("Waktu:", f"{time_g:.2f} ms")
print("Node yang dikunjungi:", len(visited_g))

print("\n=== Hasil A* ===")
print("Rute :", path_a)
print("Waktu:", f"{time_a:.2f} ms")
print("Node yang dikunjungi:", len(visited_a))

# Tabel Perbandingan
print("\n=== Perbandingan ===")
print(f"{'Algoritma':<10} | {'Waktu (ms)':<12} | {'Jumlah Node'}")
print("-" * 40)
print(f"{'GBFS':<10} | {time_g:<12.2f} | {len(visited_g)}")
print(f"{'A*':<10} | {time_a:<12.2f} | {len(visited_a)}")
