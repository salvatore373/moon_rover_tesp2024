import numpy as np
import math

def calculate_heading(x1, y1, x2, y2):
    # Compute the direction vector
    dx = x2 - x1
    dy = y2 - y1
   
    # Calculate the heading angle in radians
    theta_radians = math.atan2(dy, dx)
   
    # Convert the angle to degrees
    theta_degrees = math.degrees(theta_radians)
   
    return theta_degrees

def calculate_headings_from_path(path_coordinates):
    headings = []
    for i in range(len(path_coordinates) - 1):
        x1, y1 = path_coordinates[i]
        x2, y2 = path_coordinates[i + 1]
        heading = calculate_heading(x1, y1, x2, y2)
        headings.append(heading)
    return headings

# Example usage:
# Suppose you have a numpy array of path coordinates
path_coordinates = np.array([
    [0, 0],
    [1, 1],
    [2, 3],
    [3, 3],
    [4, 2]
])

headings = calculate_headings_from_path(path_coordinates)
print("Headings (in degrees):", headings)