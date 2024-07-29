import math

from CCVPE.models import CVM_OxfordRobotCar as CVM

import numpy as np
import torch


class Localizer:
    """
    A utility class that localizes the robot in the given map.
    """

    def __init__(self):
        """
        Create a Localizer object.
        """
        model_path = "/Volumes/SALVATORE R/Università/TESP/model/models/OxfordRobotCar/model.pt"
        # # 1
        # self.model = torch.load(model_path)
        # # 2
        self.model = CVM(device=torch.device('cpu'))
        self.model.load_state_dict(torch.load(model_path))
        #
        # self.model.eval()

    def get_pose_in_map(self, ground_img: torch.Tensor, aerial_img, previous_location=None):
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

        # DEBUG
        # THe images are the result of PIL.Image.open (RGB) passed to torchvision transforms
        # https://github.com/tudelft-iv/CCVPE/blob/main/datasets.py#L285
        # https://github.com/tudelft-iv/CCVPE/blob/main/train_OxfordRobotCar.py#L49

        _, heatmap, ori, _, _, _, _, _, matching_score_stacked6 = self.model(
            ground_img, aerial_img)

        # TODO: multiply the positions around the previous one by a value that is proportional
        #  to the distance from the previous location
        #  or
        #  consider only a rectangular interval of heatmap, centered in the previous location

        if previous_location is not None:
            # Search for the next position in a rectangular interval of the previous location
            interval_half_dim = int(512 * 0.1)
            interval_height_start, interval_height_end = previous_location[0] - interval_half_dim, previous_location[
                0] + interval_half_dim
            interval_width_start, interval_width_end = previous_location[1] - interval_half_dim, previous_location[
                1] + interval_half_dim
            current_pred = heatmap[0, :, interval_height_start:interval_height_end,
                           interval_width_start:interval_width_end]
        else:
            current_pred = heatmap[0, :, :, :]

        loc_pred = np.unravel_index(current_pred.argmax(), current_pred.shape)

        cos_pred, sin_pred = ori[0, :, loc_pred[1], loc_pred[2]]
        cos_pred = cos_pred.detach().numpy()
        sin_pred = sin_pred.detach().numpy()
        if np.abs(cos_pred) <= 1 and np.abs(sin_pred) <= 1:
            a_acos_pred = math.acos(cos_pred)
            if sin_pred < 0:
                angle_pred = math.degrees(-a_acos_pred) % 360
            else:
                angle_pred = math.degrees(a_acos_pred)
        else:
            angle_pred = 0

        # DEBUG display the heatmap
        import matplotlib.pyplot as plt
        data = heatmap[0, :, :, :].detach().numpy()[0]
        # Create a heatmap
        plt.imshow(data, cmap='viridis', interpolation='nearest')
        if previous_location is not None:
            # Extract the x and y coordinates of the points
            points = np.array([
                [interval_width_start, interval_height_end],
                [interval_width_start, interval_height_start],
                [interval_width_end, interval_height_start],
                [interval_width_end, interval_height_end]
            ])
            # Create the rectangle by closing the loop on the points
            rectangle_coords = np.append(points, [points[0]], axis=0)

            # Plot the rectangle
            plt.plot(rectangle_coords[:, 1], rectangle_coords[:, 0], color='red', linestyle='-', linewidth=2)
        # Add a color bar to show the scale
        plt.colorbar()
        # Display the heatmap
        plt.show()

        return loc_pred, angle_pred
