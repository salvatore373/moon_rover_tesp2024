import cv2
import numpy as np
from PIL import Image

from mapping.gridmap import GridMap
from mapping.mapping import Mapping
from path_planning.path_planner import PathPlanner
from path_planning.vehicle import Vehicle


def take_picture_with_camera():
    """Returns the picture taken with the connected camera."""
    return np.array(Image.open("/Volumes/SALVATORE R/Università/TESP/data/map0_resized2.jpg"))  # DEBUG

    cam = cv2.VideoCapture(0)
    _, frame = cam.read()
    return frame


def build_map(rover: Vehicle):
    """
    Takes a picture with the camera and converts is to a gridmap to be used in path planning.
    Returns the built GridMap.
    """
    mapping = Mapping()
    map_img_from_camera = take_picture_with_camera()
    mapping.get_map_homography(map_img_from_camera)
    warped_img = mapping.apply_homography_to_map(np.array(map_img_from_camera))
    grid = mapping.get_gridmap(warped_img)

    gridmap = GridMap(grid, 1.98, 1)
    gridmap.inflate_obstacles(rover)

    # infl = gridmap.inflate_obstacles(rover)
    # import matplotlib.pyplot as plt
    # plt.imshow(warped_img)
    # plt.show()
    # from mapping.gridmap import CellType
    # import matplotlib.pyplot as plt
    # test_grid = np.zeros(grid.shape)
    # test_grid[grid == CellType.OBSTACLE] = 100
    # plt.imshow(test_grid)
    # plt.show()
    # import matplotlib.pyplot as plt
    # test_grid = np.zeros(infl.shape)
    # test_grid[infl == CellType.OBSTACLE] = 100
    # plt.imshow(test_grid)
    # plt.show()

    return gridmap


def main():
    # Build the robot that we are going to use
    rover = Vehicle(0.2, 0.2, 0.3)

    # Get the map where the robot is
    map = build_map(rover)

    # Build the path planner
    path_planner = PathPlanner(map, rover)
    start = (111, 20)
    goal = (280, 135)
    best_path = path_planner.compute_best_path(start, goal)
    print(best_path)


if __name__ == '__main__':
    main()

    print()
