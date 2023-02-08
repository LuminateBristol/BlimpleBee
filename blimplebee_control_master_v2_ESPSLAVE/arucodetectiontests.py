# Import required packages
import cv2
import math
import os
import pickle

# We create the dictionary object. Aruco has some predefined dictionaries.
# (DICT_4X4_100, DICT_4X4_1000, DICT_4X4_250, DICT_4X4_50 = 0, .... , DICT_7X7_1000)
# We are going to create a dictionary, which is composed by 250 markers.
# Each marker will be of 5x5 bits (DICT_7X7_250):
aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

# Create parameters to be used when detecting markers:
parameters = cv2.aruco.DetectorParameters_create()

# Create video capture object 'capture' to be used to capture frames from the first connected camera:
capture = cv2.VideoCapture(1)

if capture is None or not capture.isOpened():
       print('Warning: unable to open video source:')

while True:
    # Capture frame by frame from the video capture object 'capture':
    ret, frame = capture.read()

    # We convert the frame to grayscale:
    #gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # lists of ids and the corners beloning to each id# We call the function 'cv2.aruco.detectMarkers()'
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dictionary, parameters=parameters)

    if not len(corners) == 0 and ids[0] == 1:
        x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
        y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

        x = x_sum / 4
        y = y_sum / 4

        heading = math.atan2(corners[0][0][0][1] - corners[0][0][2][1], corners[0][0][0][0] - corners[0][0][2][0]) + math.pi

        print(x, y, heading)


    # Draw detected markers:
    frame = cv2.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids, borderColor=(0, 255, 0))

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything:
capture.release()
cv2.destroyAllWindows()