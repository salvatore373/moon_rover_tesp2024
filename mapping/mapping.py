import cv2
import numpy as np
from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation
from PIL import Image

import torch
from scipy.spatial import distance

from path_planning.path_planner import OBSTACLE_COST

#  If the probability of a certain pixel to contain an obstacle is above this threshold, then it contains an obstacle
OBSTACLES_PROBABILITY_THRESHOLD = 0.2
# The number of pixels to consider when looking for the ones with the highest probability to contain the rover
NUM_VALUABLE_PIXELS_ROVER_DETECTION = 50


class Mapping:
    def __init__(self):
        self.processor = CLIPSegProcessor.from_pretrained("CIDAS/clipseg-rd64-refined")
        self.model = CLIPSegForImageSegmentation.from_pretrained("CIDAS/clipseg-rd64-refined")

        self.map_homography = None

    def get_objects_position(self, map_img, prompts: list[str]):
        """
        Given the image of the map, returns the position (coordinates in the image) of the objects
         in the given image.
        :param map_img: The image of the map where to recognize rocks.
        :param prompts: The labels of the objects to find in the given image.
        :return: A JSON containing the objects labels as keys and as value a matrix where each pixel corresponds to
         the probability that the object is in that pixel.
        """
        inputs = self.processor(text=prompts, images=[map_img] * len(prompts), padding="max_length",
                                return_tensors="pt")
        with torch.no_grad():
            outputs = self.model(**inputs)
        preds = outputs.logits.unsqueeze(1)

        return {label: preds[i][0] for i, label in enumerate(prompts)}

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

    def _find_sandbox_corners(self, sand_prob):
        """
        Given the probability that each pixel contains sand, it returns the position of the
         corners of the sandbox.
        :param sand_prob: A matrix representing the probability that each pixel contains sand.
        :return: The position of the corners of the sandbox.
        """
        # Make the image binary
        sand_prob = torch.sigmoid(sand_prob).numpy()
        sand_prob[sand_prob < 0.1] = 0
        sand_prob[sand_prob >= 0.1] = 1
        greyscale_sand_rectangle = (sand_prob * 255.0).astype(np.uint8)

        # Find the corners of the sandbox as contours
        contours, _ = cv2.findContours(greyscale_sand_rectangle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Assume the largest contour corresponds to the sandbox
        contour = max(contours, key=cv2.contourArea)
        # Approximate the contour to get the corners
        epsilon = 0.09 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        # Extract the corner points
        corner_points = approx.reshape(-1, 2)

        # Display the original image and the detected corners DEBUG
        # import matplotlib.pyplot as plt
        # image = cv2.cvtColor(greyscale_sand_rectangle, cv2.COLOR_GRAY2BGR)
        # for point in corner_points:
        #     cv2.circle(image, tuple(point), 5, (0, 0, 255), -1)
        # plt.figure(figsize=(10, 10))
        # plt.imshow(image)
        # plt.title("Detected Corners")
        # plt.show()

        return corner_points

    def _find_homography_matrix(self, sand_prob):
        """
        Given the probability that each pixel contains sand, it returns the homography matrix to turn the picture of
         the rectangular map into a real rectangle.
        :param sand_prob: A matrix representing the probability that each pixel contains sand.
        :return: The homography matrix to turn the picture of the rectangular map into a real rectangle.
        """
        # Find the corners of the sandbox
        sandbox_corners = self._find_sandbox_corners(sand_prob)

        # Define the desired corners of the sandbox in the image (TL, TR, BL, BR)
        image_h, image_w = sand_prob.shape
        desired_corners = [(0, 0), (0, image_w), (image_h, 0), (image_h, image_w)]

        # Associate each of the sandbox corners to the closest desired point
        sorted_desired_corners = np.empty(
            (4, 2))  # ith elem in sorted_desired_corners corresponds to ith in sandbox_corners
        for j, s_corner in enumerate(sandbox_corners):
            min_dist = image_w + image_h
            best_assoc = (-1, -1)
            best_index = -1
            for i, d_corner in enumerate(desired_corners):
                min_dist_temp = min(distance.euclidean(s_corner, d_corner), min_dist)
                if min_dist_temp != min_dist:
                    min_dist = min_dist_temp
                    best_assoc = d_corner
                    best_index = i

            # Associate this desired point to the closest sandbox corner
            sorted_desired_corners[j, :] = best_assoc
            # Prevent from associating this desired point to another sandbox corner
            del desired_corners[best_index]
        del desired_corners

        # Find the homography transformation from the source points to the destination ones
        hom_mat, _ = cv2.findHomography(sandbox_corners, sorted_desired_corners, cv2.RANSAC, 5.0)
        return hom_mat

    def get_map_homography(self, map_img):
        """
        Returns the homography matrix to transform the given image to a bird-eye view.
        :param map_img: The image of the map where the transformation should be applied.
        :return: The homography matrix for the transformation.
        """
        if self.map_homography is None:
            sand_position = self.get_objects_position(map_img, ['sand or rover'])
            map_homography = self._find_homography_matrix(sand_position['sand or rover'])

            # # Apply the homography transformation DEBUG
            # import matplotlib.pyplot as plt
            # image = cv2.imread("/Volumes/SALVATORE R/Università/TESP/data/map0.jpeg")
            # image = cv2.resize(image, (352, 352))
            # # Apply the homography transformation
            # rectified_image = cv2.warpPerspective(np.array(image), homography, image.shape[:2])
            # # Display the original and rectified images
            # plt.figure(figsize=(10, 5))
            # plt.subplot(1, 2, 1)
            # plt.title("Original Image")
            # plt.imshow(cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB))
            # plt.subplot(1, 2, 2)
            # plt.title("Rectified Image")
            # plt.imshow(cv2.cvtColor(rectified_image, cv2.COLOR_BGR2RGB))
            # plt.show()

            self.map_homography = map_homography

        return self.map_homography

    def apply_homography_to_map(self, image):
        """
        Given an image of the mao, applies the needed transformations to turn it into a bird-eye view.
        :param image: The image to transform.
        :return: The transformed image.
        """
        assert self.map_homography is not None, "You first have to call get_map_homography()."

        image = cv2.resize(image, (352, 352))
        # return cv2.warpPerspective(np.array(image), self.map_homography, image.shape[:2])
        return cv2.warpPerspective(image, self.map_homography, image.shape[:2])

    def get_gridmap(self, map_image):
        """
        Given an image, returns a grid where the cells can have 2 possible values: one for the cells containing
         obstacles and the other for free cells.
        Regardless of the size of map_image, the returned gridmap will have dimension (352, 352).
        :param map_image: The image of the map.
        :return: The gridmap. Dimension (352, 352).
        """
        # Find the position of the obstacles in the map
        obstacles_position_probability = self.get_objects_position(map_image, ['stone'])
        # Generate a map from the found obstacles
        return self.convert_obstacles_prob_to_gridmap(obstacles_position_probability['stone'])
