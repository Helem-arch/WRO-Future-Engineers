# noinspection PyUnresolvedReferences
import RPi.GPIO as GPIO
# noinspection PyUnresolvedReferences
import cv2 as cv
import numpy as np
import time


def signal_detection(img, signal_size, weight, object_size, focal_distance, px):
    blurred = cv.medianBlur(img, 15)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    height, width = img.shape[:2]

    # Setting up the first mask
    lower_limit = np.array([15, 97, 125])
    upper_limit = np.array([35, 143, 225])
    mask1 = cv.inRange(hsv, lower_limit, upper_limit)
    kernel = np.ones((11, 11), np.uint8)
    mask1 = cv.dilate(mask1, kernel, iterations=2)
    mask1 = cv.erode(mask1, kernel, iterations=2)

    # Setting up the second mask
    lower_limit = np.array([0, 150, 100])
    upper_limit = np.array([10, 225, 225])
    mask2 = cv.inRange(hsv, lower_limit, upper_limit)
    kernel = np.ones((11, 11), np.uint8)
    mask2 = cv.dilate(mask2, kernel, iterations=2)
    mask2 = cv.erode(mask2, kernel, iterations=2)

    # Finding the contours and marking them
    contours1, _ = cv.findContours(mask1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours1:
        if cv.contourArea(cnt) > height * width * 0.03:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(img, "Yellow", (x, y), font, 0.5, (255, 255, 255), 2, cv.LINE_AA)
            distance = (focal_distance * signal_size) / (h * px) + weight
            print("Yellow Object distance :", distance, "cm")
            if cx < (width // 2 - object_size // 2):
                return True, True  # The first argument refers to the colour of the signal and the second one refers
                # to whether the object is on the right side
            else:
                return True, False  # # The first argument refers to the colour of the signal and the second one
                # refers to whether the object is on the right side

    contours2, _ = cv.findContours(mask2, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours2:
        if cv.contourArea(cnt) > height * width * 0.03:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(img, "Orange", (x, y), font, 0.5, (255, 255, 255), 2, cv.LINE_AA)
            distance = (focal_distance * signal_size) / (h * px) + weight
            print("Orange Object distance :", distance, "cm")
            if cx > (width // 2 + object_size // 2):
                return False, False  # The first argument refers to the colour of the signal and the second one
                # refers to whether the object is on the right side
            else:
                return False, True  # The first argument refers to the colour of the signal and the second one refers
                # to whether the object is on the right side


def d_l(sx, sy, object_size, f, window_size):
    y = sy * 0.0264583333  # Converting to cm
    distance_from_object = int((f * object_size) / y) * 2
    x = ((window_size // 2) - sx) * 0.0264583333  # Converting to cm
    rx = int((window_size // 2) - ((x * distance_from_object) / f))
    return distance_from_object, rx


def wall_detection(img, wall_size, lower_range, upper_range, focal_length):
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img = cv.GaussianBlur(img, (5, 5), 10, 10)
    _, thresh = cv.threshold(img, 10, 255, cv.THRESH_BINARY_INV)

    filtered = np.argwhere(thresh > 0)
    filtered, screen_y = np.unique(filtered, return_counts=True, axis=0)
    filtered = np.hsplit(filtered, 2)
    if filtered[0].max() > filtered[1].max():
        screen_x, screen_y = np.unique(filtered[0], return_counts=True)
        filter_arr = (screen_y > lower_range) & (screen_y < upper_range)
        screen_x = screen_x[filter_arr]
        screen_y = screen_y[filter_arr]
    else:
        screen_x, screen_y = np.unique(filtered[1], return_counts=True)
        filter_arr = (screen_y > lower_range) & (screen_y < upper_range)
        screen_x = screen_x[filter_arr]
        screen_y = screen_y[filter_arr]

    dis, ax = d_l(screen_x.max(), screen_y.min(), wall_size, focal_length, thresh.shape[0])
    d_l_graph = np.zeros((dis + 1, ax + 1), np.uint8)
    empty = np.zeros((dis + 1, ax + 1, 3), np.uint8)

    x1 = screen_x[0]
    y1 = screen_y[0]
    x2 = screen_x[0]
    y2 = screen_y[0]
    line_m = []
    line_points = []
    for i in range(len(screen_x)):
        dis, ax = d_l(screen_x[i], screen_y[i], wall_size, focal_length, thresh.shape[0])
        d_l_graph[dis, ax] = 255
        if abs(dis - y2) < 20 and (ax - x2) < 20:
            x2 = ax
            y2 = dis
        else:
            cv.line(empty, (x2, y2), (x1, y1), (0, 255, 255), 2)
            if (x2 - x1) != 0:
                line_m.append(0 - ((y2 - y1) / (x2 - x1)))
                line_points.append((x2, y2))
            x1 = ax
            y1 = dis
            x2 = ax
            y2 = dis
    if (x2 - x1) != 0:
        cv.line(empty, (x2, y2), (x1, y1), (0, 255, 255), 2)
        line_m.append(0 - ((y2 - y1) / (x2 - x1)))

    distances = []
    ay, ax = d_l(0, thresh.shape[1], wall_size, focal_length, thresh.shape[0])
    for i in range(len(line_m)):
        m = line_m[i]
        distances.append(ax - (m * line_points[i][0]))


def right(pwm, in1, in2, rotation):
    if rotation != 2:
        rotation += 1
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(100)
        time.sleep(1)
        pwm.ChangeDutyCycle(0)


def left(pwm, in1, in2, rotation):
    if rotation != 0:
        rotation -= 1
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwm.ChangeDutyCycle(100)
        time.sleep(1)
        pwm.ChangeDutyCycle(0)
