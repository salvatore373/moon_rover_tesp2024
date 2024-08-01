import cv2
import numpy as np
from PIL import Image

from color_tag.colorTagDev import tagFinder
from mapping.gridmap import GridMap
from mapping.mapping import Mapping
from path_planning.path_planner import PathPlanner
from path_planning.vehicle import Vehicle
from rover_code.pid import PDController, calculate_heading


def take_picture_with_camera():
    """Returns the picture taken with the connected camera."""
    # return np.array(Image.open("/Volumes/SALVATORE R/Università/TESP/data/map0_resized2.jpg"))  # DEBUG

    # import pyrealsense2 as rs
    # pipe = rs.pipeline()
    # profile = pipe.start()
    # try:
    #     for i in range(0, 100):
    #         frames = pipe.wait_for_frames()
    #         for f in frames:
    #             print(f.profile)
    # finally:
    #     pipe.stop()

    # import ecapture as ec
    # ec.capture(0, False, "/Users/salvatore/Desktop/Università/TESP2024/moon_rover_tesp2024/frame.jpg")
    # return np.array(Image.open("/Users/salvatore/Desktop/Università/TESP2024/moon_rover_tesp2024/frame.jpg"))
    # frame = np.array(Image.open("/Users/salvatore/Desktop/Università/TESP2024/moon_rover_tesp2024/frame.jpg"))
    # import matplotlib.pyplot as plt
    # plt.imshow(frame)
    # plt.show()
    # return frame

    cam = cv2.VideoCapture(0)
    _, frame = cam.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = cv2.resize(frame, (round(0.8 * frame.shape[1]), round(0.8 * frame.shape[0])))
    cam.release()

    import matplotlib.pyplot as plt
    plt.imshow(frame)
    plt.show()
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
    import matplotlib.pyplot as plt
    plt.imshow(warped_img)
    plt.show()
    from mapping.gridmap import CellType
    import matplotlib.pyplot as plt
    test_grid = np.zeros(grid.shape)
    test_grid[grid == CellType.OBSTACLE] = 100
    plt.imshow(test_grid, cmap='gray_r')
    plt.show()
    # import matplotlib.pyplot as plt
    # test_grid = np.zeros(infl.shape)
    # test_grid[infl == CellType.OBSTACLE] = 100
    # plt.imshow(test_grid)
    # plt.show()

    return gridmap, mapping


def main():
    # Build the robot that we are going to use
    rover = Vehicle(0.2, 0.2, 0.3)

    # Get the map where the robot is
    map, mapping = build_map(rover)

    # Build the path planner
    path_planner = PathPlanner(map, rover)
    start = (111, 20)
    goal = (280, 135)
    best_path = path_planner.compute_best_path(start, goal)
    print(best_path)

    # Initialize the control part
    controller = PDController(Kpx=1, Kdx=0.1, Kpy=1, Kdy=0.1, Kp_theta=1, Kd_theta=0.1, Kv=1, Komega=1, d=0.15)
    curr_x, curr_y = best_path[0]
    des_x, des_y = best_path[1]
    # Initialize the localizer
    localizer = tagFinder()

    while curr_x != des_x and des_y != curr_y:
        img = take_picture_with_camera()
        img = mapping.apply_homography_to_map(img)

        # Compute control commands
        theta = calculate_heading(x1=curr_x, x2=des_x, y1=curr_y, y2=des_y)
        ((robot_x, robot_y, robot_orient), _, _, _) = localizer.tagAngle(img)
        v_FL, v_FR, v_R = controller.compute_control(x_d=des_x, y_d=des_y, theta_d=theta, x=robot_x,
                                                     y=robot_y, theta=robot_orient, dt=0.1)
        print("Front Left motor velocity:", v_FL)
        print("Front Right motor velocity:", v_FR)
        print("Rear motor velocity:", v_R)

    # TODO: stop when an obstacle is too close


if __name__ == '__main__':
    main()
