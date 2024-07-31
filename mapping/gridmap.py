from enum import Enum

import cv2
import numpy as np

from path_planning.vehicle import Vehicle


class CellType(Enum):
    """
    Enumeration of types of cells that can be in the grid passed to the path planner.
    """
    FREE = 0
    OBSTACLE = 1


class GridMap:
    """
    A class to define a real-world map as a grid.
    """

    def __init__(self, grid, original_width, original_height):
        """
        Initializes a GridMap object.
        :param grid: The grid representing the map.
        """
        self.grid = grid

        self.original_width = original_width  # meters
        self.original_height = original_height  # meters
        self.grid_width = grid.shape[1]  # cells
        self.grid_height = grid.shape[0]  # cells

        # Compute the resolution as res = num_meters / num_cells
        self.resolution = self.original_height / self.grid_height
        # assert self.original_width / self.grid_width == self.resolution

    def update_gridmap(self, new_gridmap):
        self.grid = new_gridmap

    def _meters_to_cells(self, meters: float) -> int:
        """
        Convert the given number of meters into the corresponding number of cells.
        num_cells = num_meters / resolution
        """
        return round(meters / self.resolution)

    def _cells_to_meters(self, cells: float) -> float:
        """
        Convert the given number of cells into the corresponding number of meters.
        num_cells = num_meters / resolution
        """
        return cells * self.resolution

    def inflate_obstacles(self, grid, vehicle: Vehicle) -> np.array:
        """
        Increase the size of the obstacles by the dimension of the vehicle's width, to be sure
         that the vehicle always fits in the space between 2 obstacles.
        :param grid: The grid in which to find the path.
        :param vehicle: The vehicle that will be in the map.
        :return: Returns the inflated grid.
        """
        # Initialize the inflated grid
        inflated_grid = np.copy(grid)

        # Get the number of safe-cells to add around each obstacle
        safe_cells_num = self._meters_to_cells(vehicle.width / 2)

        w, h = grid.shape
        for i in range(w):
            for j in range(h):
                # If the current cell in the original grid is an obstacle, add some obstacles around it
                if grid[i, j] == CellType.OBSTACLE:
                    inflated_grid[max(0, i - safe_cells_num): min(w, i + safe_cells_num + 1),
                    max(0, j - safe_cells_num): min(h, j + safe_cells_num + 1)] = CellType.OBSTACLE

        self.grid = inflated_grid
        return inflated_grid
