from enum import Enum
from typing import Any
import matplotlib.pyplot as plt

import networkx as nx
import numpy as np
import scipy
from numpy import floating

from path_planning.vehicle import Vehicle

# The cost that a cell containing an obstacle should have
OBSTACLE_COST = 100
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
        self.grid_resolution = grid_resolution  # TODO sandbox = 1m x 2.05m

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
        Increase the size of the obstacles by the dimension of the vehicle's width, to be sure
         that the vehicle always fits in the space between 2 obstacles.
        :param grid: The grid in which to find the path.
        :return: Returns the inflated grid.
        """
        # Initialize the inflated grid
        inflated_grid = np.copy(grid)

        # Get the number of safe-cells to add around each obstacle
        safe_cells_num = self._meters_to_cells(self.vehicle.width)

        w, h = grid.shape
        for i in range(w):
            for j in range(h):
                # If the current cell in the original grid is an obstacle, add some obstacles around it
                if grid[i, j] == CellType.OBSTACLE:
                    inflated_grid[max(0, i - safe_cells_num): min(w, i + safe_cells_num + 1),
                    max(0, j - safe_cells_num): min(h, j + safe_cells_num + 1)] = CellType.OBSTACLE

        return inflated_grid

    @staticmethod
    def _compute_heuristics(node_a, node_b) -> floating:
        """
        The heuristic function to use in the path planning algorithm
        :param node_a: The current node.
        :param node_b: The goal node.
        :return: A measure of the distance from the current node to the goal node.
        """
        (x1, y1) = node_a
        (x2, y2) = node_b
        # return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

        diff_vec = np.array([x2 - x1, y2 - y1])
        return np.linalg.norm(diff_vec)

    @staticmethod
    def _convert_grid_to_graph(grid: np.array) -> nx.Graph:
        """
        Converts the given grid into a networkx graph.
        :param grid: The grid to convert.
        :return: The resulting networkx graph.
        """

        graph = nx.grid_graph(dim=grid.shape)
        nx.set_edge_attributes(graph, {e: grid[e[1]] for e in graph.edges()}, "cost")
        return graph

    @staticmethod
    def _display_path(grid, path):
        # Create a plot
        fig, ax = plt.subplots()
        # Display the grid using a greyscale colormap
        cax = ax.matshow(grid, cmap='gray_r')
        # Add a colorbar for reference
        fig.colorbar(cax)
        # Plot the line connecting the cells
        path_x = [x for x, y in path]
        path_y = [y for x, y in path]
        ax.plot(path_y, path_x, marker='o', color='red', linestyle='-', linewidth=2, markersize=5)
        # Invert the y-axis to match the usual matrix indexing
        ax.invert_yaxis()
        # Set gridlines
        ax.set_xticks(np.arange(-.5, grid.shape[1], 1), minor=True)
        ax.set_yticks(np.arange(-.5, grid.shape[0], 1), minor=True)
        ax.grid(which='minor', color='black', linestyle='-', linewidth=2)
        # Show the plot
        plt.show()

    def compute_best_path(self, grid, start, goal) -> list[tuple[int, int]]:
        """
        Returns the best path to move the robot from start to goal in grid.
        :param grid: A 2D grid represented as a 2D numpy array that represents the environment where
         the robot has to move. The cells are either CellType.OBSTACLE or CellType.FREE.
        :param start: The coordinate of the cell of the robot's starting position.
        :param goal: The coordinate of the cell that the robot has to reach.
        :return: The sorted list of the coordinates of the cells that belong to the path,
         from the start position to the goal.
        """

        # Inflate obstacles
        inflated_grid = self._inflate_obstacles(grid)

        # Compute grid's cost
        grid_cost = self._add_grid_cost(inflated_grid)

        # Convert the grid to a graph to be used in A star
        graph = self._convert_grid_to_graph(grid_cost)

        path = nx.astar_path(graph, start, goal, heuristic=self._compute_heuristics, weight="cost")

        # DEBUG
        # self._display_path( np.vectorize(lambda cell: OBSTACLE_COST if cell == CellType.OBSTACLE else 0)(grid), path)
        # self._display_path( grid_cost, path)

        return path
