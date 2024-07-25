from enum import Enum

import numpy as np
import scipy

from path_planning.vehicle import Vehicle

# The cost that a cell containing an obstacle should have
OBSTACLE_COST = 1
# The cost that a free cell should have
FREE_COST = 0


class CellType(Enum):
    """
    Enumeration of types of cells that can be in the grid passed to the path planner.
    """
    FREE = 0
    OBSTACLE = 1


class PathPlanner:
    """
    A utility function to compute the best path to move a car-like robot in the given grid
    """

    def __init__(self, vehicle: Vehicle, grid_resolution: float = 0.5):
        """
        Initialize the path planner.
        :param vehicle: The vehicle to move in the grid.
        :param grid_resolution: The conversion ratio between cells and meters. (res = meters / num_cells)
        """
        super().__init__()
        self.vehicle = vehicle
        self.grid_resolution = grid_resolution

    def _meters_to_cells(self, meters: float) -> int:
        """
        Convert the given number of meters into the corresponding number of cells.
        num_cells = num_meters / resolution
        """
        return round(meters / self.grid_resolution)

    def _cells_to_meters(self, cells: float) -> float:
        """
        Convert the given number of cells into the corresponding number of meters.
        num_cells = num_meters / resolution
        """
        return cells * self.grid_resolution

    def _add_grid_cost(self, grid: np.array):
        """
        In each cell of the given grid, add the respective cost. In other terms, it computes the cost function
         associated to this grid.
        :param grid: The grid whom the cost function has to be computed.
        :return: A grid of the same type and dimension of the given one, where each cell contains its cost.
        """
        w, h = grid.shape

        # Initialize the cost grid
        assert FREE_COST == 0, ("Since the cost of a free cell is not 0, the cost_grid should be initialized with"
                                " a different value")
        cost_grid = np.zeros((w, h), dtype=float)
        cost_grid[grid == CellType.OBSTACLE] = OBSTACLE_COST

        # Define a measure of the distance that the path should keep from the obstacles
        sigma = 2
        # Expand the cost of the obstacles in the neighboring cells, in order to keep the path
        # distant from the obstacles.
        return scipy.ndimage.gaussian_filter(cost_grid, sigma=sigma)

    def _inflate_obstacles(self, grid) -> np.array:
        """
        Increase the size of the obstacles by the biggest dimension of the vehicle, to be sure
         that the vehicle always fits in the space between 2 obstacles.
        :param grid: The grid in which to find the path.
        :return: Returns the inflated grid.
        """
        # Initialize the inflated grid
        inflated_grid = np.copy(grid)

        # Get the number of safe-cells to add around each obstacle
        max_vehicle_dim = max(self.vehicle.width, self.vehicle.height)
        safe_cells_num = self._meters_to_cells(max_vehicle_dim)

        w, h = grid.shape
        for i in range(w):
            for j in range(h):
                # If the current cell in the original grid is an obstacle, add some obstacles around it
                if grid[i, j] == CellType.OBSTACLE:
                    inflated_grid[max(0, i - safe_cells_num): min(w, i + safe_cells_num + 1),
                        max(0, j - safe_cells_num): min(h, j + safe_cells_num + 1)] = CellType.OBSTACLE

        return inflated_grid

    def compute_best_path(self, grid, start, goal):  # TODO: define return type
        """
        Returns the best path to move the robot from start to goal in grid.
        :param grid: A 2D grid represented as a 2D numpy array that represents the environment where
         the robot has to move. The cells are either CellType.OBSTACLE or CellType.FREE.
        :param start: The coordinate of the cell of the robot's starting position.
        :param goal: The coordinate of the cell that the robot has to reach.
        :return: TODO:
        """

        # Inflate obstacles
        inflated_grid = self._inflate_obstacles(grid)

        # Compute grid's cost
        grid_cost = self._add_grid_cost(inflated_grid)
