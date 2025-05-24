from planning.path_planner import astar

# 0 = free, 1 = obstacle
grid = [
  [0, 0, 0, 0, 0, 0],
  [0, 1, 1, 1, 0, 0],
  [0, 0, 0, 1, 0, 0],
  [0, 0, 0, 1, 0, 0],
  [0, 0, 0, 0, 0, 0]
]

start = (0,0)
goal = (5, 4)

path = astar(grid, start, goal)
print("Path:", path)