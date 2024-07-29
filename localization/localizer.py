import math

import numpy as np
from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation


class Localizer:
    """
    A utility class that localizes the robot in the given map.
    """

    def __init__(self):
        pass

    def get_pose_in_map(self, ground_img, aerial_img, previous_location=None):
        """
        Given ground-view and an aerial-view images of the same map, returns
         the pose of the robot that is in the given map and took the ground-view image.
        :param ground_img: The ground view of the map, taken by the robot. The dimension must be (154, 231).
        :param aerial_img: The aerial view of the whole map. The dimension must be (512, 512).
        :param previous_location: The location of the robot in the map according to the previous measurement
         (format: height, width). If provided, the position of the robot will be searched in a rectangular
          interval of this position in the map.
        :return: The pose of the robot that took the ground-view image of the map in the given aerial view.
        """
        pass
