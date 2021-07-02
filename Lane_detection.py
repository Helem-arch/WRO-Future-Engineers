import numpy as np
import cv2 as cv
import time


def find_lines(line_details):
    rho, theta = line_details[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    px1 = int(x0 + 1000 * -b)
    py1 = int(y0 + 1000 * a)
    px2 = int(x0 - 1000 * -b)
    py2 = int(y0 - 1000 * a)
    return px1, px2, py1, py2


start = time.time()
img = cv.imread(
    "C:\\Users\\Jose T\\PycharmProjects\\opencv_project\\Sources\\Lane too left.jpg")  # Frame which you want
hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
lower_limit = np.array([0, 0, 0])
upper_limit = np.array([0, 0, 0])
thresh = cv.inRange(hsv, lower_limit, upper_limit)
kernel = np.ones((9, 9), np.uint8)
thresh = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)
lines = cv.HoughLines(thresh, 1, np.pi / 180, 200)
line_slopes = []
line_m = []
for line in lines:
    x1, x2, y1, y2 = find_lines(line)
    if round((x2 - x1) / (y2 - y1)) not in line_slopes:
        cv.line(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
        line_slopes.append(round((x2 - x1) / (y2 - y1)))
        line_m.append((x2 - x1) / (y2 - y1))

if abs(line_m[0]) == abs(line_m[1]):
    print("centered")
else:
    if abs(line_m[0]) > abs(line_m[1]):
        print(line_m[0] > 0, ", True means too close to the left")
    else:
        print(line_m[1] > 0, ", True means too close to the left")
end = time.time()
print("Slopes :", line_m)
print("Time taken :", end - start)
cv.imshow("Thresh", thresh)
cv.waitKey(0)
