import numpy as np
from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation
from PIL import Image

import torch

from path_planning.path_planner import OBSTACLE_COST

#  If the probability of a certain pixel to contain an obstacle is above this threshold, then it contains an obstacle
OBSTACLES_PROBABILITY_THRESHOLD = 0.2
# The number of pixels to consider when looking for the ones with the highest probability to contain the rover
NUM_VALUABLE_PIXELS_ROVER_DETECTION = 50


class Mapping:
    def __init__(self):
        self.processor = CLIPSegProcessor.from_pretrained("CIDAS/clipseg-rd64-refined")
        self.model = CLIPSegForImageSegmentation.from_pretrained("CIDAS/clipseg-rd64-refined")
        self.prompts = ["stone", "sand", "rover"] # TODO: check whether they are all needed

    def get_obstacles_position(self, map_img):
        """
        Given the image of the map, returns the position (coordinates in the image) of the rocks in the given image.
        :param map_img: The image of the map where to recognize rocks.
        :return: The
        """
        inputs = self.processor(text=self.prompts, images=[map_img] * len(self.prompts), padding="max_length",
                                return_tensors="pt")

        with torch.no_grad():
            outputs = self.model(**inputs)

        preds = outputs.logits.unsqueeze(1)

        return {label: preds[i][0] for i, label in enumerate(self.prompts)}

    def convert_obstacles_prob_to_gridmap(self, obstacles_prob):
        """
        Takes in input the probability that the corresponding pixel contains an obstacle and returns a map
         where the cells can be either 100 if they contain an obstacle, or 0 if they don't.
        :param obstacles_prob: A matrix containing the probability that the corresponding pixel contains an obstacle.
        :return: A grid where the cells can be either 100 if they contain an obstacle, or 0 if they don't.
        """
        obstacles_grid = torch.sigmoid(obstacles_prob).numpy()
        obstacles_grid[obstacles_grid < OBSTACLES_PROBABILITY_THRESHOLD] = 0
        obstacles_grid[obstacles_grid != 0] = OBSTACLE_COST

        # Show the map as grid DEBUG
        # import matplotlib.pyplot as plt
        # import matplotlib.colors as mcolors
        # cmap = mcolors.ListedColormap(['white', 'black'])
        # bounds = [0, 50, 100]
        # norm = mcolors.BoundaryNorm(bounds, cmap.N)
        # # Display the image
        # plt.imshow(obstacles_grid, cmap=cmap, norm=norm)
        # # Add gridlines for better visualization
        # plt.grid(which='both', color='grey', linestyle='-', linewidth=0.5)
        # # Add a color bar to show the scale (optional)
        # # plt.colorbar(ticks=[0, 100], label='Value')
        # # Show the plot
        # plt.show()

        return obstacles_grid

    def find_rover_position(self, rover_prob):
        """
        Takes in input the probability that the corresponding pixel contains the rover and returns the coordinates of
         one point of the rover in the given matrix.
        :param rover_prob: A matrix representing the probability that the corresponding pixel contains the rover.
        :return: The coordinates of one point of the rover in the given matrix
        """
        rover_prob = torch.sigmoid(rover_prob).numpy()
        # Get the coordinates of the 50 pixels with the highest value
        # Flatten the array and get the indices of the top 50 values
        flat_indices = np.argpartition(rover_prob.flatten(), -NUM_VALUABLE_PIXELS_ROVER_DETECTION)[
                       -NUM_VALUABLE_PIXELS_ROVER_DETECTION:]
        # Convert flat indices to 2D coordinates
        coords = np.array(np.unravel_index(flat_indices, rover_prob.shape)).T
        # Compute the mean of the most likely points to contain the rover
        return np.mean(coords, axis=0)

    def get_map_from_image(self):
        # perform perspective warping
        # warped_img = ... TODO
        warped_img = Image.open("/Volumes/SALVATORE R/Università/TESP/data/map2.jpeg")  # DEBUG

        # find the position of obstacles, rover and sandbox
        obstacles_position_probability = self.get_obstacles_position(warped_img)

        # Generate a map from the found obstacles
        self.convert_obstacles_prob_to_gridmap(obstacles_position_probability['stone'])

        # turn warped image into grid containing obstacles (using the whole warped img as map)
        # return ...(obstacles_position_probability, warped_img.size) TODO
        pass
