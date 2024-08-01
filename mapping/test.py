import numpy as np

from mapping import Mapping

from PIL import Image

from gridmap import GridMap
from path_planning.vehicle import Vehicle


def test_mapping():
    mapping = Mapping()
    map_img_from_camera = np.array(Image.open("/Volumes/SALVATORE R/Università/TESP/data/map0_resized.jpg"))  # DEBUG
    mapping.get_map_homography(map_img_from_camera)
    warped_img = mapping.apply_homography_to_map(np.array(map_img_from_camera))
    grid = mapping.get_gridmap(warped_img)

    # import matplotlib.pyplot as plt
    # plt.imshow(warped_img)
    # plt.show()
    #
    # import matplotlib.pyplot as plt
    # from gridmap import GridMap, CellType
    # test_grid = np.zeros(grid.shape)
    # test_grid[grid == CellType.OBSTACLE] = 100
    # plt.imshow(test_grid)
    # plt.show()

    gridmap = GridMap(grid, 1.98, 1)
    rover = Vehicle(0.2, 0.2, 0.3)
    print('Inflating obstacles...')
    infl = gridmap.inflate_obstacles(rover)
    print('Finished inflating obstacles')

    # import matplotlib.pyplot as plt
    # from gridmap import CellType
    # test_grid = np.zeros(infl.shape)
    # test_grid[infl == CellType.OBSTACLE] = 100
    # plt.imshow(test_grid)
    # plt.show()


if __name__ == '__main__':
    test_mapping()
