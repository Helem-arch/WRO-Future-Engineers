import RPi.GPIO as GPIO
import cv2 as cv
import numpy as np
import time
from math import atan

# ################## This program contains all the functions used by the main program in the WRO competition ################## *


# This function detects whether the signal is on the correct side of the car and the angle it would have to turn to move to the right side
# It has the d_l function as its helper function 

def signal_detection(image, signal_size, weight, object_size, focal_distance, px, l):
    img = image
    blurred = cv.medianBlur(img, 15)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    height, width = img.shape[:2]

    # Setting up the first mask
    lower_limit = np.array([25, 150, 40])
    upper_limit = np.array([85, 230, 255])
    mask1 = cv.inRange(hsv, lower_limit, upper_limit)
    kernel = np.ones((11, 11), np.uint8)
    mask1 = cv.dilate(mask1, kernel, iterations=2)
    mask1 = cv.erode(mask1, kernel, iterations=2)

    # Setting up the second mask
    lower_limit = np.array([97, 170, 70])
    upper_limit = np.array([180, 255, 255])
    mask2 = cv.inRange(hsv, lower_limit, upper_limit)
    kernel = np.ones((11, 11), np.uint8)
    mask2 = cv.dilate(mask2, kernel, iterations=2)
    mask2 = cv.erode(mask2, kernel, iterations=2)

    # Finding the contours and marking them
    contours1, _ = cv.findContours(mask1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[-2:]
    for cnt in contours1:
        if cv.contourArea(cnt) > height * width * 0.012:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(img, "Green", (x, y), font, 0.5, (255, 255, 255), 2, cv.LINE_AA)
            dis, latx = d_l(x+w//2, h, signal_size, focal_distance, l)
            print("Green Object lateral distance :", latx, "cm")
            print("Green Object distance :", dis, "cm")
            try:
                angle = 100 - (180/np.pi*(atan(dis/abs(latx))))
                print("Degrees to turn :", 100 - (180/np.pi*(atan(dis/abs(latx)))))
            except ZeroDivisionError:
                angle = 30
                print("Degrees to turn :", 30)
            if angle < 40:
                angle = 40
            if dis < 60:
                if cx < (width // 2 - object_size // 2):
                    return img, [1, 1, angle, dis, latx]  # The first argument refers to the colour of the signal and the second one refers
                    # to whether the object is on the right side
                else:
                    return img, [1, 0, angle, dis, latx] # # The first argument refers to the colour of the signal and the second one
                    # refers to whether the object is on the right side

    contours2, _ = cv.findContours(mask2, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[-2:]
    for cnt in contours2:
        if cv.contourArea(cnt) > height * width * 0.012:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(img, "Red", (x, y), font, 0.5, (255, 255, 255), 2, cv.LINE_AA)
            dis, latx = d_l(x+w//2, h, signal_size, focal_distance, l)
            print("Red Object lateral distance :", latx, "cm")
            print("Red Object distance :", dis, "cm")
            try:
                angle = 100 - (180/np.pi*(atan(dis/abs(latx))))
                print("Degrees to turn :", angle)
            except ZeroDivisionError:
                angle = 30
                print("Degrees to turn :", 30)
            if angle < 40:
                angle = 40
            if dis < 60:
                if cx > (width // 2 + object_size // 2):
                    return img, [0, 1, angle, dis, latx]  # The first argument refers to the colour of the signal and the second one
                    # refers to whether the object is on the right side
                else:
                    return img, [0, 0, angle, dis, latx]  # The first argument refers to the colour of the signal and the second one refers
                    # to whether the object is on the right side
    return img, [2, 0, 0]


# This function returns the distance and lateral distance of the signal from the car 
def d_l(sx, sy, object_size, f, window_size):
    y = sy * 0.0264583333  # Converting to cm
    distance_from_object = int((f * object_size) / y)
    x = ((window_size // 2) - sx) * 0.0264583333  # Converting to cm
    rx = int(((x * distance_from_object) / f))
    return distance_from_object, rx


# This function converts the points of the Hough line transform to actual coordinates in the window 
def get_real_coords(x1, x2, y1, y2, l, b):
    m = (y2 - y1) / (x2 - x1)
    coords = []
    coords.append(int((m*x1 - y1 + 0) / m))
    coords.append(int(y1 - m*(x1 - 0)))
    coords.append(int((m*x1 - y1 + b) / m))
    coords.append(int(y1 - m*(x1 - l)))
    for i in range(len(coords)):
        if i % 2 == 0:
            if coords[i] < 0:
                coords[i] = 0
            if coords[i] > l:
                coords[i] = 0
        else:
            if coords[i] < 0:
                coords[i] = 0
            if coords[i] > b:
                coords[i] = 0          
    rx1 = 0
    ry1 = 0
    rx2 = 0
    ry2 = 0 
    for i in range(len(coords)):
        if i == 0:
            if coords[i] != 0:
                rx1 = coords[i]
                ry1 = 1
        if i == 1:
             if coords[i] != 0:
                if ry1 == 0:
                    rx1 = 1
                    ry1 = coords[i]
                else:
                    rx2 = 1
                    ry2 = coords[i]
        if i == 2:
            if coords[i] != 0:
                if rx1 == 0:
                    rx1 = coords[i]
                    ry1 = b
                else:
                    rx2 = coords[i]
                    ry2 = b
        if i == 3:
            if coords[i] != 0:
                if ry1 == 0:
                    rx1 = l
                    ry1 = coords[i]
                else:
                    rx2 = l
                    ry2 = coords[i]
    return rx1, ry1, rx2, ry2


# This function returns the points detected by the Hough line transform
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


# This function warps the image in order to remove perspective effects from the image, enabling us to make
# better judgements about the image 
def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv.warpPerspective(img, matrix, (w, h))
    return imgWarp


# This function determines the slope of the track lines and determines the angle the car would have to turn
# in order to avoid hitting the wall. This function has the get_real_coords, find_lines and warpImg functions
# its helper functions 
def wall_detection(image, l, b, threshold):
    points = np.float32([(13, 73), (l - 13, 73),
                         (0, 118), (l - 0, 118)])
    
    img = warpImg(image, points, l, b)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
     
    # Mask to subtract the blue and orange lines
    lower_limit = np.array([25, 150, 40])
    upper_limit = np.array([85, 230, 255])
    mask1 = cv.inRange(hsv, lower_limit, upper_limit)
    kernel = np.ones((11, 11), np.uint8)
    mask1 = cv.dilate(mask1, kernel, iterations=2)
    mask1 = cv.erode(mask1, kernel, iterations=2)
    
    img1_bg1 = cv.bitwise_and(img,img,mask = mask1)
    img2_fg = cv.bitwise_and(img2,img2,mask = mask)
    
    lower_limit = np.array([97, 170, 70])
    upper_limit = np.array([180, 255, 255])
    mask2 = cv.inRange(hsv, lower_limit, upper_limit)
    kernel = np.ones((11, 11), np.uint8)
    mask2 = cv.dilate(mask2, kernel, iterations=2)
    mask2 = cv.erode(mask2, kernel, iterations=2)
    
    edges = cv.Canny(img, 100, 200)
    lines = cv.HoughLines(edges,1,np.pi/180, 50)
    
    right_wall = 0
    left_wall = 0
    front_wall = 0
    line_slopes = []
    fx1, fx2, fy1, fy2, rx1, rx2, ry1, ry2, lx1, lx2, ly1, ly2 = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    
    if lines is not None:
        for line in lines:
            try:
                x1, x2, y1, y2 = find_lines(line)
                angle = 180/np.pi*atan(1/((y2 - y1) / (x2 - x1)))
                if round(angle) not in line_slopes:
                    cv.line(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    line_slopes.append(round(angle))
                    if angle > 45 or angle < -45:
                        fx1, fx2, fy1, fy2 = x1, x2, y1, y2
                        front_wall = angle
                    elif angle > 0:
                        rx1, rx2, ry1, ry2 = x1, x2, y1, y2
                        right_wall = angle
                    else:
                        lx1, lx2, ly1, ly2 = x1, x2, y1, y2
                        left_wall = angle
            except:
                pass
    
    if left_wall != 0 - threshold and left_wall != 0:
        llx1, lly1, llx2, lly2 = get_real_coords(lx1, lx2, ly1, ly2, l, b)
        P1_disy = 20 * (1-(lly1/b)) + 20
        P2_disy = 20 * (1-(lly2/b)) + 20
        P1_disx = 20 * (llx1/l)
        P2_disx = 20 * (llx2/l)
        m = (P1_disy - P2_disy) / (P1_disx - P2_disx)
        line_dis = (m * P2_disx - P2_disy + 0) / m
        if abs(line_dis) < 10:
            return img, "R", abs(left_wall)
        
    if right_wall != threshold and right_wall != 0:
        llx1, lly1, llx2, lly2 = get_real_coords(rx1, rx2, ry1, ry2, l, b)
        P1_disy = 20 * (1-(lly1/b)) + 20
        P2_disy = 20 * (1-(lly2/b)) + 20
        P1_disx = 20 * (llx1/l)
        P2_disx = 20 * (llx2/l)
        m = (P1_disy - P2_disy) / (P1_disx - P2_disx)
        line_dis = ((m * P1_disx - P1_disy + 0) / m) - 15
        if abs(line_dis) < 10:
            return img, "L", abs(right_wall)
        
    if front_wall != 0:
        llx1, lly1, llx2, lly2 = get_real_coords(fx1, fx2, fy1, fy2, l, b)
        
        P1_dis = 20 * (1-(lly1/l)) + 20
        P2_dis = 20 * (1-(lly2/l)) + 20
        if (P1_dis + P2_dis)/2 < 35:
            return img, "F",  abs(front_wall)
        
    return img, "N", 0

# This function helps the vehicle to perform the manuever inorder to turn the car in the right angle towards the right 
def right_angle(pwm_1, pwm_2, IN1, IN2, IN3, IN4, rotationi, rotation_angle, motor_power, speed):
    rotation = centered(pwm_1, IN2, IN1, rotationi, motor_power, speed)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    rotation = left(pwm_1, IN2, IN1, rotation, pwm_2, motor_power, speed)
    time.sleep(motor_power*(rotation_angle/45))
    rotation = centered(pwm_1, IN2, IN1, rotation, motor_power, speed)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(motor_power)
    rotation = right(pwm_1, IN2, IN1, rotation, pwm_2, motor_power, speed)
    time.sleep(motor_power*(rotation_angle/45))
    rotation = centered(pwm_1, IN2, IN1, rotation, motor_power, speed)
    return rotation

# This function helps the vehicle to perform the manuever inorder to turn the car in the right angle towards the left 
def left_angle(pwm_1, pwm_2, IN1, IN2, IN3, IN4, rotationi, rotation_angle, motor_power, speed):
    rotation = centered(pwm_1, IN2, IN1, rotationi, motor_power, speed)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    rotation = right(pwm_1, IN2, IN1, rotation, pwm_2, motor_power, speed)
    time.sleep(motor_power*(rotation_angle/45))
    rotation = centered(pwm_1, IN2, IN1, rotation, motor_power, speed)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(motor_power)
    rotation = left(pwm_1, IN2, IN1, rotation, pwm_2, motor_power, speed)
    time.sleep(motor_power*(rotation_angle/45))
    rotation = centered(pwm_1, IN2, IN1, rotation, motor_power, speed)
    return rotation


# Turns the car to the right   
def right(pwm, in1, in2, rotation, pwm_2, motor_power, speed):
    if rotation != 2:
        rotation += 1
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(speed)
        time.sleep(motor_power*0.3)
        pwm.ChangeDutyCycle(0)
    return rotation


# Turns the car to the left
def left(pwm, in1, in2, rotation, pwm_2, motor_power, speed):
    if rotation != 0:
        rotation -= 1
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwm.ChangeDutyCycle(speed)
        time.sleep(motor_power*0.3)
        pwm.ChangeDutyCycle(0)
    return rotation


# Centeres the steering wheels of the car
def centered(pwm, in1, in2, rotation, motor_power, speed):
    if rotation == 2:
        rotation -= 1
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwm.ChangeDutyCycle(speed)
        time.sleep(motor_power*0.2)
        pwm.ChangeDutyCycle(0)
    if rotation == 0:
        rotation += 1
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(speed)
        time.sleep(motor_power*0.3)
        pwm.ChangeDutyCycle(0)
        
    return rotation


# Moves the car back
def back(in3, in4, time):
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    time.sleep(time)

    
# Moves the car front  
def forward(in3, in4, sec):
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    time.sleep(sec)
