import numpy as np
import cv2
from colorTagDev import tagFinder

def test_colortag():
    tf = tagFinder()
    imRaw = cv2.imread("color_tag/ref.jpg")
    cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
    cv2.imshow("image1", imRaw)
    cv2.waitKey(0)
    (pos,tagsFound,contourImg,rectMap) = tf.tagAngle(imRaw)
    print(pos)
    cv2.imshow("image1", np.hstack([imRaw, contourImg, rectMap]))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    test_colortag()