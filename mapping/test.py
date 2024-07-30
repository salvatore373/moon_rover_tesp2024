import numpy as np

from mapping import Mapping

from PIL import Image


def test_mapping():
    mapping = Mapping()
    map_img_from_camera = Image.open("/Volumes/SALVATORE R/Università/TESP/data/map0.jpeg")  # DEBUG
    mapping.get_map_homography(map_img_from_camera)
    warped_img = mapping.apply_homography_to_map(np.array(map_img_from_camera))
    gridmap = mapping.get_gridmap(warped_img)
    print()


if __name__ == '__main__':
    test_mapping()
