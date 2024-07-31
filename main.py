import cv2


def take_picture_with_camera():
    """Returns the picture taken with the connected camera."""
    cam = cv2.VideoCapture(0)
    _, frame = cam.read()
    return frame


def init():
    pass


def main():
    pass


if __name__ == '__main__':
    main()

    print()
