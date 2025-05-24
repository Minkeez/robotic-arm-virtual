from planning.path_planner import astar, generate_real_world_grid, cm_to_grid, grid_to_cm

grid = generate_real_world_grid()

start_cm = (20, 20)
goal_cm = (100, 80)

start_grid = cm_to_grid(*start_cm)
goal_grid = cm_to_grid(*goal_cm)

path_grid = astar(grid, start_grid, goal_grid)

if path_grid:
  path_cm = [grid_to_cm(x, y) for (x, y) in path_grid]
  print("Path (cm):", path_cm)
else:
  print("No path found")