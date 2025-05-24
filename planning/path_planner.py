import heapq

GRID_SIZE_CM = 5 # 5 cm resolution
WORKSPACE_CM = 120
GRID_WIDTH = WORKSPACE_CM // GRID_SIZE_CM # 24

class Node:
  def __init__(self, position, parent=None):
    self.position = position # (x, y)
    self.parent = parent
    self.g = 0 # cost from start
    self.h = 0 # heuristic to goal
    self.f = 0 # total cost

  def __lt__(self, other):
    return self.f < other.f # Allow comparison for heapq

def astar(grid, start, goal):
  open_set = []
  start_node = Node(start)
  goal_node = Node(goal)
  heapq.heappush(open_set, (start_node.f, start_node))
  closed_set = set()

  while open_set:
    _, current = heapq.heappop(open_set)

    if current.position == goal:
      return reconstruct_path(current)
    
    closed_set.add(current.position)

    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,-1),(1,-1),(-1,1)]:
      neighbor_pos = (current.position[0] + dx, current.position[1] + dy)

      if not in_bounds(neighbor_pos, grid) or grid[neighbor_pos[1]][neighbor_pos[0]] == 1:
        continue
      if neighbor_pos in closed_set:
        continue

      neighbor = Node(neighbor_pos, current)
      neighbor.g = current.g + 1
      neighbor.h = euclidean(neighbor.position, goal_node.position)
      neighbor.f = neighbor.g + neighbor.h

      heapq.heappush(open_set, (neighbor.f, neighbor))
  
  return None # no path found

def in_bounds(pos, grid):
  x, y = pos
  return 0 <= x < len(grid[0]) and 0 <= y < len(grid)

def euclidean(p1, p2):
  return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) ** 0.5

def reconstruct_path(node):
  path = []
  while node:
    path.append(node.position)
    node = node.parent

  return path[::-1] # reverse

def generate_real_world_grid(width=24, height=24):
  grid = [[0 for _ in range(width)] for _ in range(height)]

  # Add obstacles (1 = blocked)
  # Example: obstacle block in center
  for y in range(10, 14):
    for x in range(10, 14):
      grid[y][x] = 1
  
  return grid

def cm_to_grid(x_cm, y_cm, resolution=5):
  return int(x_cm // resolution), int(y_cm // resolution)

def grid_to_cm(x_grid, y_grid, resolution=5):
  return x_grid * resolution, y_grid * resolution