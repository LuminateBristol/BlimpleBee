"""
Height tests:
HHickson
Sept 2022

A testing script to play with and understand the following:
- Sending http requests to the ESP32 access point
- OpenCV aruco marker detection and manipulation
- Feedback loop to provide localisation information to the ESP32 OR control from Python via a simple PID controller
"""

import requests
import time
import numpy as np
import cv2
import cv2.aruco as aruco

def exception_handler(request, exception):
    print("Request failed")

def moveupanddown():
    speedd = str(200)
    speedfwd = str(100)
    off = str(0)

    while True:
        res = requests.get('http://192.168.4.1/rmf', params=speedd)
        #res = requests.get('http://192.168.4.1/rmf', params=speedfwd)
        #res = requests.get('http://192.168.4.1/lmf', params=speedfwd)
        print(res.text)

        time.sleep(5)

        res = requests.get('http://192.168.4.1/rmf', params=off)
        #res = requests.get('http://192.168.4.1/rmb', params=speedfwd)
        #res = requests.get('http://192.168.4.1/lmb', params=speedfwd)
        print(res.text)

        time.sleep(5)

moveupanddown()