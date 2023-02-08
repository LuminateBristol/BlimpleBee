"""
Blimplebee control Python master
Version: v2
Sept 2022
Henry Hickson

Off-board master controller for the BlimpleBee.
Whilst the aim is for Python to handle localisaation requests only nad all other computation be on the robot.
This Python master has been setup for initial testing because:
- No need for constant re-upload of code to BlimpleBee
- Vision system can stay on throughout the experimentation
- PID tuning is much faster

The ESP controller has been shown to work and the PID values can be used back on that system once properly tuned.
"""


from bottle import run, request, post, route
import math
import time
import cv2
import math
import requests
import matplotlib.pyplot as plt
import sys

class PID:
    """
    PID controller class.
    """
    def __init__(self, k_p, k_i, k_d):
        """
        Initialisation of class variables
        """
        self.P = 0
        self.I = 0
        self.D = 0
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.error = 0
        self.error_old = 0
        self.etdt = 0
        self.dt = 0
        self.oldtime = 0
        self.newtime = 0

    def update_pid(self, demand, measurement):
        """
        Update PID values
        :param demand: target value
        :param measurement: current value
        :param k_p: proportional weighting
        :param k_i: integral weighting
        :param k_d: derivative weighting
        :return:
        """
        self.error = demand - measurement
        self.newtime = round(time.time() * 1000)
        self.oldtime = self.newtime
        self.dt = self.newtime - self.oldtime

        self.etdt = self.etdt + self.error*self.dt
        ediff = self.error - self.error_old
        self.error_old = self.error

        self.P = self.k_p*self.error
        self.I = self.k_i*self.etdt
        if self.dt != 0:
            self.D = self.k_d * (ediff / self.dt)
        else:
            self.D = 0

        return self.P + self.I + self.D


class BlimpleBeeControl:
    """
    Class to control the movement of the BlimpleBee drone
    Covers the control of individual motors as well as heading, altitude and waypoint control
    """

    def __init__(self, pidheading, piddistance, pidturnonspot, pidheight, pidheightspeed):
        """
        Initiator
        :param pidheading: pid controller class for heading
        :param piddistance: pid controller class for distance
        """
        self.pidturnonspot = pidturnonspot
        self.pidheading = pidheading
        self.piddistance = piddistance
        self.pidheight = pidheight
        self.pidheightspeed = pidheightspeed
        self.motorspeedr = 0
        self.motorspeedl = 0
        self.motorspeeda = 0
        self.oldheight = 0
        self.oldtime = 0

        self.timerecord = []
        self.datarecord = []

    def motorcontrol(self, motor, direction, motorspeed):
        """
        Function to control the motors
        :param motor: which motor - string input "right" "left" or "altitude"
        :param direction: bool - 1 forward, 0 backward
        :param motorspeed: pwm speed input
        :return:
        """
        mdir = "null"
        if motor == "right":
            if direction == 1:
                mdir = "rmf"
            else:
                mdir = "rmb"
        if motor == "left":
            if direction == 0:
                mdir = "lmf"
            else:
                mdir = "lmb"
        if motor == "altitude":
            if direction == 0:
                mdir = "mmd"
            else:
                mdir = "mmu"

        reqmsg = mdir + str(motorspeed)

        res = requests.get('http://192.168.4.1/', params=reqmsg)

    def stopmotors(self):
        """
        Stop all motors from running
        :return:
        """
        self.motorcontrol("left", 0, 0)
        self.motorcontrol("right", 0, 0)
        self.motorcontrol("altitude", 0, 0)

    def headingcontrol(self, oldheading, heading):
        """
        Function to move to a given heading.
        Heading defined as the global heading as given by the camera frame of reference.
        :param heading:
        :return:
        """
        rotationspeed = self.pidturnonspot.update_pid(heading, oldheading)

        if rotationspeed > 0:
            self.motorspeedl = rotationspeed
            self.motorspeedr = -rotationspeed
            self.motorcontrol("left", 1, abs(self.motorspeedl))
            self.motorcontrol("right", 0, abs(self.motorspeedr))
            # print("leftspin", rotationspeed)
            # print("rightspin", rotationspeed)
        else:
            self.motorspeedl = -rotationspeed
            self.motorspeedr = rotationspeed
            self.motorcontrol("left", 0, abs(self.motorspeedl))
            self.motorcontrol("right", 1, abs(self.motorspeedr))
            # print("leftspin", rotationspeed)
            # print("rightspin", rotationspeed)

    def waypointcontrol(self, x, y, heading, xnew, ynew): # TODO: Not tuned or verified
        """
        Function to control waypoint following motion of the robot
        :param x: existing x position
        :param y: existing y position
        :param xnew: target x position
        :param ynew: target y position
        :return:
        """
        headingtarget = math.atan2(ynew-y, xnew-x) + math.pi
        distarget = math.sqrt((xnew - x)**2 + (ynew - y)**2)

        # Check to see if spin on the spot or move forwards:
        if headingtarget - heading > math.pi/4 or headingtarget - heading < -math.pi/4:
            # Spin on the spot
            self.headingcontrol(heading, math.degrees(headingtarget))
        else:
            # Move forwards
            pid_distance = self.piddistance.update_pid(0, distarget)
            print("distarget: ", distarget)
            # print("pid distance: ", pid_distance)
            if abs(pid_distance) > 100:  # Top out at 100 to avoid excessive speeds (for testing)
                pid_distance = 100

            pid_heading = self.pidheading.update_pid(math.degrees(headingtarget), math.degrees(heading))
            if pid_heading > 150:
                pid_heading = 150
            elif pid_heading < -150:
                pid_heading = -150

            self.motorspeedl = abs(pid_distance) - pid_heading
            self.motorspeedr = abs(pid_distance) + pid_heading

            if self.motorspeedl >= 0:    # TODO: put in the motor control section!!!
                dirl = 0
            else:
                dirl = 1
            if self.motorspeedr >= 0:
                dirr = 0
            else:
                dirr = 1

            # Use a combination of pid_distance and pid_heading for on the fly directional updates
            self.motorcontrol("left", dirl, abs(self.motorspeedl))
            self.motorcontrol("right", dirr, abs(self.motorspeedr))

        print("location now:", x, y, math.degrees(heading))
        print("waypoint", xnew, ynew, math.degrees(headingtarget), distarget)
        print("")
        print("left", self.motorspeedl)
        print("right", self.motorspeedr)

    def altitudecontrol(self, altitude, targetaltitude):
        """
        Height control function
        In this case the rate of change of altitude is calculated at each timestep and the aim is to reach zero
        at the target height.
        :param height: current height
        :param targetheight: target height
        """
        dadt = (altitude - self.oldaltitude) / (time.time() - self.oldtime)
        self.oldaltitude = altitude
        self.oldaltiude = time.time()

        targetspeed = self.pidheight.update_pid(targetaltitude, altitude)  # Decreases as we get closer to target - use the gains here to get a good negative slope on the target pwm as it gets closer to the target
        targetpwm = self.pidheightspeed.update_pid(targetspeed, dadt)      # Decreases as we get closer to target
        self.motorspeeda = self.motorspeeda + targetpwm

        if self.motorspeeda > 220:
            self.motorspeeda = 220
        elif self.motorspeeda < -150:
            self.motorspeeda = -150

        print("Altitude is: ", altitude)
        """
        print("targetspeedpid: ", targetspeed)
        print("dadt is: ", dadt)
        print("TargetpwmPID is: ", targetpwm)
        print("Altpwm is: ", self.motorspeeda)
        """

        if self.motorspeeda > 0:
            self.motorcontrol("altitude", 1, self.motorspeeda)
        else:
            self.motorcontrol("altitude", 0, abs(self.motorspeeda))


class Run:
    """
    Class to run the robot.
    """
    def __init__(self, BlimpleBeeControl, cameranumber):
        """
        Initialise.
        """
        self.i = 0
        self.state = 1
        self.previousstate = self.state
        self.timenow = round(time.time() * 1000)
        self.BlimpleBeeControl = BlimpleBeeControl
        self.cameranumber = cameranumber
        self.datarecord1 = []
        self.datarecord2 = []
        self.timerecord = []

    def robotcontrol(self, x, y, heading, testnum):
        """
        Run the controller.
        This is inserted into the vision code to run movement commands whilst the vision system is up and running.
        This is currently setup for testing the Blimp. Each test has a testnum index which allows us to switch between.
        These tests are setup to gain some results ready for a conference paper to be written September 2022.
        This can be edited as needed to allow for different movements to be tested.
        :return:
        """
        numtests = 5
        int = 30000

        if testnum == 1:
            # Battery testing
            self.randommovementtest()

        elif testnum == 2:
            # Single height testing
            # Test the robot's ability to hover at a single height / altitude
            heighttarget = 290
            if (round(time.time()) * 1000) - self.timenow > int:
                self.timenow = round(time.time() * 1000)
                self.BlimpleBeeControl.heightcontrol(x, heighttarget)

        elif testnum == 3:
            # Altitude change testing
            # This tests how well the robot moves between two altitudes using PID control
            # All altitudes are measured in pixels as taken from the OpenCV library
            if (self.state % 2) == 0:
                print("going to 550")
                self.BlimpleBeeControl.altitudecontrol(x, 550)
                if (round(time.time() * 1000)) - self.timenow > int:
                    self.state = self.state + 1
                    self.timenow = (round(time.time() * 1000))
            else:
                print("going to 290")
                self.BlimpleBeeControl.altitudecontrol(x, 290)
                if (round(time.time() * 1000)) - self.timenow > int:
                    self.state = self.state + 1
                    self.timenow = (round(time.time() * 1000))

            if self.state > self.previousstate:
                print("Test number:", self.state)
                print(self.timerecord)
                print(self.datarecord1)
                print(self.datarecord2)
                self.previousstate = self.state

            if self.state == numtests*2:
                # If we have done x number of repetitions of up and down movements, print the chart and stop
                self.BlimpleBeeControl.stopmotors()
                sys.exit("End of test")

        elif testnum == 4:
            # Altitude speed testing
            pwminput = 100
            self.BlimpleBeeControl.motorcontrol("altitude", 1, pwminput)
            self.timerecord.append(time.time())
            self.datarecord1.append(x)

            if x > 550:
                self.BlimpleBeeControl.stopmotors()
                print(self.timerecord)
                print(self.datarecord1)

        elif testnum == 5:
            int = 100
            # Straight line to a waypoint test
            if (round(time.time()) * 1000) - self.timenow > int:
                self.timenow = round(time.time() * 1000)
                if self.state == 1:
                    print("going to WP1")
                    xtar = 500
                    ytar = 250
                    self.BlimpleBeeControl.waypointcontrol(x, y, heading, xtar, ytar)
                    self.timerecord.append(x)
                    self.datarecord1.append(y)

                    tol = 50
                    if xtar-tol < x < xtar+tol and ytar-tol < y < ytar+tol:

                        self.BlimpleBeeControl.stopmotors()
                        print(self.datarecord1)
                        print(self.datarecord2)
                        self.handledata("save", "plot", 0)


    def randommovementtest(self):
        """
        Do random movement around the flyinglab to test battery life when moving constantly.
        """
        straightint = 20000
        turnint = 2000
        pwm = 150
        if self.i == 0:
            print(round(time.time() * 1000) - self.timenow)
            self.BlimpleBeeControl.motorcontrol("left", 0, pwm)
            self.BlimpleBeeControl.motorcontrol("right", 0, pwm-round(pwm*0.17))
            print("straight")


    def runwithvision(self):
        """
        Run the controller with an active vision window.
        Uses the OpenCV to track the robot and an active window to show the user what is happening.
        Implements the robotcontrol() function for controlling the movement - change path within that function only.
        :param: cameranumber - the input camera index
        """
        aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

        # Create parameters to be used when detecting markers:
        parameters = cv2.aruco.DetectorParameters_create()

        # Create video capture object 'capture' to be used to capture frames from the first connected camera:
        capture = cv2.VideoCapture(self.cameranumber)
        window_name = "frame"
        """
        cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        """

        if capture is None or not capture.isOpened():
            print('Warning: unable to open video source:')

        while True:
            # Capture frame by frame from the video capture object 'capture':
            ret, frame = capture.read()

            # We convert the frame to grayscale:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # lists of ids and the corners beloning to each id# We call the function 'cv2.aruco.detectMarkers()'
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_frame, aruco_dictionary, parameters=parameters)

            if not len(corners) == 0 and ids[0] == 1:
                x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

                x = x_sum / 4
                y = y_sum / 4

                heading = math.atan2(corners[0][0][0][1] - corners[0][0][2][1], corners[0][0][0][0] - corners[0][0][2][0]) + math.pi

                # print(x, y, math.degrees(heading))
                self.robotcontrol(x, y, heading, 5)

                # Draw detected markers:
                frame = cv2.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids, borderColor=(0, 255, 0))

                self.robotcontrol(x, y, heading, 1)

            # Display the resulting frame
            cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


# Run the code
if __name__ == "__main__":

    PidTurnOnSpot = PID(0.5, 0, 1)  # For turning to heading

    PidHeading = PID(0.75, 0, 5)  # For going to waypoint
    PidDistance = PID(0.25, 0, 5)

    PidHeight = PID(0.15, 0, 0)    # By decreasing kp here, the balloon should slow down earlier on as it will become negative in pwm earlier
    PidHeightSpeed = PID(0.25, 0, 10)   # By incresing kp here, the balloon should slow down quicker as the change in pwm will be more rigorous

    BBControl = BlimpleBeeControl(PidHeading, PidDistance, PidTurnOnSpot, PidHeight, PidHeightSpeed)

    cameranumber = 1
    RunRobot = Run(BBControl, cameranumber)
    RunRobot.runwithvision()



