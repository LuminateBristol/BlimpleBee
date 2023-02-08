# BlimpleBee - Helium Assisted Drones
This repository stores the code used for the BlimpleBee project, a University of Bristol masters robotics project.

The code here is designed for testing the robot only. 

## Code log:
1. blimplebee_control_master_v1_ESPMASTER - setup such that the ESP32 is the main controller of the robot. All code runs on the robot. This code has not been fully tested and was not used for the 2022 project.

2. blimplebee_control_master_v2_ESPSLAVE - setup such that a Python module running on a nearby computer is the main controller of the robot. This code is written in Python only and sends commands to the ESP32 via WiFi. The corresponding ESP32 code is given in 3. esp32_motor_ESPSLAVE.

3. esp32_motor_ESPSLAVE - a simple WiFi program to set up a wireless access point on the robot, receive commands via any connected device and interpret those commands. Commands currently setup relate only to moving each of the three motors back or forward by a given PWM value.

## Guide to running the robot:
This guide details how to operate the robot in SLAVE mode, as used for all testing in the 2022 project.

Step 1: Upload the Arduino code to the robot: esp32_motor_ESPSLAVE

Step 2: Turn on the robot (attach battery)

Step 3: Connect to the ESP32 wireless access point: Username: BlimpleBee Password: 123456789

Optional step: Test the connection by sending a command to the robot. To do this we simply type the IP address of the robot into an internet browser followed by the command we wish to send. The IP address is: 192.168.4.1. To turn the right motor forward for example, we would enter: 192.168.4.1/rmf0100. 'rmf' stands for 'right motor forward', '0100' means turn to PWM = 100. Note maximum PWM = 255.

Step 4: Control from Python. Once the Arduino code is setup, Python can simply send commands to the robot using the IP address over WiFi. For an example set of code see 'Blimplebee_control_python_master_v2.py' - this was used for testing in the 2022 masters project so is made for experimental purposes. 

The main class for control in this code is the 'BlimpleBeeControl' class. This sends messages to the robot to control left, right and altitude motors. A simple PID controller is included in a seperate class.

There is also a 'Run' class which was used to run programs for testing the robot. This can still be used but is not elegantly written and was mostly used for testing purposes so it is recommended that users write their own 'Run' class for this purpose.
