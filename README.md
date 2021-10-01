# WRO-Future-Engineers
This repository contains the code that was used for creating the autonomous vehicle for the competition

## Modules used:
In the code the main module used is OpenCV (and numpy for Opencv). It is a well known module used in reputed computer vision programs. It is used in the Raspberry Pi to process the information from the camera to detect the traffic signals as well as the colour of the signals. It is also used for the wall detection program as well as the lap detection programs.

The other modules used include:
1. Picamera and picamera.array module - Inorder to get the frame from the picamera
2. Time - to get the time

## Program functioning:
The program consists of a main "Game code" which runs the program, it also contains another file called "utlis" which contains all the functions used in the program. These both files have to be in the same directory for the program to work properly.

The program loop goes as follows:
1. The program detects the signals and the whether it is on the right side of the signal
In the next loop, 
1. The program detects the wall contours and approximates a line to it
2. Then program then detects the slope of the line and uses some predifined measurements of the camera range to determine the distance of the car from the line.
3. According to the distance determined it would maneuver to move away from the line
In the same loop, 
4. The program then detects the blue and orange lines and according to the order in which they occur determines the direction of the lap, and uses this information to determine the direction to turn when the car encounters a wall.

## Car electromechanical parts:
The car contains the following parts:
1. Raspberry Pi 3A - The Brains of the car which recieves the frames from the camera, processes it and then gives the commands to the motor controller to the motors.
2. Picamera v2
3. Two DC motors
4. L289N Motor controller
5. Double layered cardboard chasis as well as some plastic parts for stablilising the components

## Detailed Program functioning:
The program consists of three parts:
1. Signal Detection
2. Wall detection
3. Lap detection

### Signal detection:
1. The program first converts the RGB input from the camera to HSV format for making it easier to find the signals in the image
2. It then filters the HSV frame by using the opencv function cv.inRange() function with the help of predefined values for the green and red HSV colours.
3. It then uses the contour approximation function in the opencv module to find the contours and approximate a circle and rectangle to it
4. The centre of the circle is used to approximate the position of the signal in real life
5. The program then checks if the signal is in the correct direction and accordingly sends the signals to the motor controller for maneuvers.

![Signal detection logic](https://user-images.githubusercontent.com/84518833/135608358-edd4c591-133e-42ac-88e2-22d7efcff02f.png)


### Wall Detection:
1. The frame which is recieved from the camera is first cropped (ROI) since we only need the part of the frame where the wall is there
2. It is then converted to HSV format and the black colour of the wall is filtered with predefined limits
3. Then it is passed on to the contour approximation function after which a line is approximated to the wall
4. The slope of the line is used to determine the distance of the wall from the car
5. According to distance, the car determines whether it has to move to left or right and by how much and accordingly sends the signals to the motor controller for maneuvers.

![Wall detection](https://user-images.githubusercontent.com/84518833/135438528-99f1c214-ab96-4f96-9bec-1a942e960573.png)


### Lap detection:
1. The frame which is recieved from the camera is first cropped (ROI) since we only need the part of the frame where the lines are there
2. It is then converted to HSV format and the blue and orange lines are filtered with predefined limits  
3. Then we approximate a circle to each of the blue and orange lines and find their centres
4. If the centre of the blue line is above the orange line then direction of the lap is defined as left else it will be defined as right
5. After it detects the lines once it increments the counter and then only checks the next time atleast after 3 seconds to ensure that it does not increment the counter by seeing the same line
6. Once the counter reaches 12 the prograam is stopped after three more seconds

![Lap detection](https://user-images.githubusercontent.com/84518833/135440517-ebcbe865-63b0-40f1-9d0b-6c6d52f658e3.png)

