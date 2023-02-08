"""
 Python localisation for the BlimpleBee via http requests.
 
 The BlimplBee requests its location from the Python server through an http post request.
 Python uses OpenCV and a webcam facing the drone to calculate its position

 HHickson
 Version: v1
 Sept 2022
"""

import cv2
from bottle import run, request, post, route
import math

# Setup ArUco marker parameters
aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters_create()

# Function definition

def runhttpserver(capture):
    """
    Function to run the http server.
    The server runs continuously until stopped.
    If an http request is received, the data will be read and the response function will be triggered.
    """
    @post('/')
    def index():
        data = request.body.read()
        response = returnposition(capture)
        return(response)

    run(host='0.0.0.0', port=8090, debug=True)

def returnposition(capture):
    """
    Function to return the position of the BlimpleBee based on the aruco marker placed on it.
    The aruco marker is identified and if found, it's x-y position (in pixels) and heading is calculated.
    :return: x, y, heading returned as a string in pixel, pixel, radians
    """

    aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    # Create parameters to be used when detecting markers:
    parameters = cv2.aruco.DetectorParameters_create()

    # Create video capture object 'capture' to be used to capture frames from the first connected camera:
    capture = cv2.VideoCapture(1)

    if capture is None or not capture.isOpened():
        print('Warning: unable to open video source:')

    # Capture frame by frame from the video capture object 'capture':
    ret, frame = capture.read()

    # We convert the frame to grayscale:
    # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # lists of ids and the corners belonging to each id# We call the function 'cv2.aruco.detectMarkers()'
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dictionary, parameters=parameters)

    # Identify corner locations and compute x, y and heading
    if not len(corners) == 0:
        x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
        y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

        x = x_sum / 4
        y = y_sum / 4

        heading = math.atan2(corners[0][0][0][1] - corners[0][0][2][1], corners[0][0][0][0] - corners[0][0][2][0]) + math.pi

        print(x, y, math.degrees(heading))

        # Draw detected markers:
        # frame = cv2.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids, borderColor=(0, 255, 0))

        # Display the resulting frame
        # cv2.imshow('frame', frame)

        return ",".join([str(round(x, 2)), str(round(y, 2)), str(round(heading, 2))])

    else:
        return "Error"


# Run the code
if __name__ == "__main__":
    # Start the camera
    capture = cv2.VideoCapture(1)
    if capture is None or not capture.isOpened():
        print('Warning: unable to open video source:')

    # Run the server
    runhttpserver(capture)