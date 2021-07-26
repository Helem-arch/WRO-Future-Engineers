# noinspection PyUnresolvedReferences
import RPi.GPIO as GPIO
# noinspection PyUnresolvedReferences
import picamera
import cv2 as cv
import numpy as np
import io
import time
import utlis

GPIO.setmode(GPIO.BCM)

# Motor 1 setup (Rear driving motor)
IN1 = 1
IN2 = 2
ENA = 5

# Motor 2 setup (Front steering motor)
IN3 = 3
IN4 = 4
ENB = 6

# Setting up the outputs
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

pwm_1 = GPIO.PWM(ENA, 100)
pwm_2 = GPIO.PWM(ENB, 100)

GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
pwm_1.start(100)
pwm_2.start(0)
stop_program = False
lap_number = 0

# Certain constants
Px = 0.0264583333  # Pixel to cm conversion
Focal_distance = 20  # Approx
Signal_size = 10  # in cm
Weight = 5
Object_size = int(240 * 0.3)
upper_range = 500
lower_range = 100
Wall_size = 10

while not lap_number == 3:
    # Create a memory stream so photos doesn't need to be saved in a file
    stream = io.BytesIO()

    # Get the picture (low resolution, so it should be quite fast)
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)
        camera.capture(stream, format='jpeg')

    # Convert the picture into a numpy array
    # noinspection PyTypeChecker
    buff = np.fromstring(stream.getvalue(), dtype=np.uint8)

    # Now creates an OpenCV image
    image = cv.imdecode(buff, 1)

    signal_color, correct_side = utlis.signal_detection(image, Signal_size, Weight, Object_size, Focal_distance, Px)
    utlis.wall_detection(image, Wall_size, lower_range, upper_range, Focal_distance)
