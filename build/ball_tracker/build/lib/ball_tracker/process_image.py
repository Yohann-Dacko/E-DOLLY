import cv2
import numpy as np

import cv2
import numpy as np

def find_circles(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                                param1=50, param2=30, minRadius=10, maxRadius=100)
    out_image = image.copy()
    keypoints = []

    if circles is not None:
        circles = np.uint16(np.around(circles))
        h, w = image.shape[:2]
        for (x, y, r) in circles[0, :]:
            cv2.circle(out_image, (x, y), r, (0, 255, 0), 2)
            norm_x_min = (x - r) / w * 2 - 1
            norm_y_min = (y - r) / h * 2 - 1
            norm_x_max = (x + r) / w * 2 - 1
            norm_y_max = (y + r) / h * 2 - 1
            keypoints.append((norm_x_min, norm_y_min, norm_x_max, norm_y_max))
    return keypoints, out_image


def draw_window2(image, rect_px, color=(255, 0, 0), line=5):
    #-- Draw a rectangle from top left to bottom right corner
    return cv2.rectangle(image, (rect_px[0], rect_px[1]), (rect_px[2], rect_px[3]), color, line)

def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]
    scale = [cols, rows, cols, rows]
    return [int(a * b / 100) for a, b in zip(rect_perc, scale)]

def normalise_keypoint(cv_image, kp):
    rows = float(cv_image.shape[0])
    cols = float(cv_image.shape[1])
    center_x = 0.5 * cols
    center_y = 0.5 * rows
    x = (kp.pt[0] - center_x) / (center_x)
    y = (kp.pt[1] - center_y) / (center_y)
    return cv2.KeyPoint(x, y, kp.size / cv_image.shape[1])

def wait_on_gui():
    cv2.waitKey(2)

def no_op(x):
    pass