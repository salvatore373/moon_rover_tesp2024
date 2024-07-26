import numpy as np

from path_planning.path_planner import PathPlanner, CellType
from path_planning.vehicle import Vehicle

if __name__ == '__main__':
    rover = Vehicle(0.3, 0.3)
    path_planner = PathPlanner(rover)

    # grid sizes (in meters)
    # grid_height = 1
    # grid_width = 2

    grid = np.full((20, 20), CellType.FREE)
    # grid[4:7, 4:7] = CellType.OBSTACLE
    grid[3:5, 3:5] = CellType.OBSTACLE
    grid[7:11, 9:10] = CellType.OBSTACLE
    grid[13:14, 15:19] = CellType.OBSTACLE

    start = (0, 0)
    goal = (6, 17)

    path_planner.compute_best_path(grid, start, goal)
