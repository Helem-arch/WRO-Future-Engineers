import numpy as np
import cv2 as cv
import time

# Choosing the camera
cap = cv.VideoCapture(0)
# Setting resolution
cap.set(3, 500)
cap.set(4, 500)

object_size = 200
""" 
[ 15 113 145] [ 35 133 225] - Y
[  0 180 159] [  0 200 239] - O
[101 177  73] [121 197 153] - B
"""

while True:
    start = time.time()
    _, img = cap.read()
    blurred = cv.medianBlur(img, 15)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    height, width = img.shape[:2]

    # Setting up the first mask
    lower_limit = np.array([15, 97, 125])
    upper_limit = np.array([35, 143, 225])
    mask1 = cv.inRange(hsv, lower_limit, upper_limit)
    kernel = np.ones((11, 11), np.uint8)
    mask1 = cv.dilate(mask1, kernel, iterations=2)
    mask1 = cv.erode(mask1, kernel, iterations=1)

    # Setting up the second mask
    lower_limit = np.array([0, 150, 100])
    upper_limit = np.array([10, 225, 225])
    mask2 = cv.inRange(hsv, lower_limit, upper_limit)
    kernel = np.ones((11, 11), np.uint8)
    mask2 = cv.dilate(mask2, kernel, iterations=2)
    mask2 = cv.erode(mask2, kernel, iterations=1)

    # Filtering the masks from the image
    filtered1 = cv.bitwise_and(img, img, mask=mask1)
    filtered2 = cv.bitwise_and(img, img, mask=mask2)

    # Finding the contours and marking them
    contours1, _ = cv.findContours(mask1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours1:
        if cv.contourArea(cnt) > 3000:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(img, "Yellow", (x, y), font, 0.5, (255, 255, 255), 2, cv.LINE_AA)
            if x+w < (width // 2 - object_size // 2):
                cv.line(img, (int(cx), 0), (int(cx), height), (0, 255, 0), 2)
            else:
                cv.line(img, (int(cx), 0), (int(cx), height), (0, 0, 255), 2)

    contours2, _ = cv.findContours(mask2, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours2:
        if cv.contourArea(cnt) > 3000:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(img, 'Orange', (x, y), font, 0.5, (255, 255, 255), 2, cv.LINE_AA)
            if x > (width // 2 + object_size // 2):
                cv.line(img, (int(cx), 0), (int(cx), height), (0, 255, 0), 2)
            else:
                cv.line(img, (int(cx), 0), (int(cx), height), (0, 0, 255), 2)

    # Showing the Images
    cv.line(img, (width//2 + object_size//2, 0), (width//2 + object_size//2, height), (200, 200, 150), 5)
    cv.line(img, (width//2 - object_size//2, 0), (width//2 - object_size//2, height), (200, 200, 150), 5)
    cv.imshow("Original", img)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    end = time.time()
    print("Time taken for one frame :", end - start)

cv.destroyAllWindows()
