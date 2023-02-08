/*
   ESP 32-S2 QT-PY Big Rig motor control tests.
   Details:
   - Three motors
   - Two DRV8833 chips
   - One QT-PY ESP32-S2 microcontroller

   Author: Henry Hickson
   Date: August 2022
*/

#include "esphttprequest.h"
#include "motorcontrol.h"
#include "pid.h"

// Setup httprequest class
EspHttpRequest esphttp;
const char* ssid = "henphone";
const char* password =  "hennerz123";
const char* ipaddress = "http://192.168.43.136:8090/";
String requestername = "BlimpleBee";

// Setup motor control class
MotorControl motors;
int movement_status = 0;

// Setup the array to host the pose of the robot (to avoid awkward memory allocation stuff)
float pose[3];

// Setup PID controller parameters
PID_c pidheading;
PID_c piddistance;

float PID_heading = 0;
float K_P_heading = 30;
float K_I_heading = 0;
float K_D_heading = 0;
float PID_distance = 0;
float K_P_distance = 1;
float K_I_distance = 0;
float K_D_distance = 0;

// Time parameters
unsigned long current_time = millis();

void setup() {
  Serial.begin(115200);

  // Wifi Setup
  esphttp.initiatehttpwifi(ssid, password);


  // Motors setup
  motors.motorsetup();
}

void loop() {
  Serial.println("running");
  getpose();
  delay(2000);
  gotowaypoint(150,150);
  delay(2000);
  gotowaypoint(300,300);
}

void getpose() {
  // Function to request pose from Python server running ArUco marker pose detection
  // String message received from Python is converted into a array output
  // return: pose - array - [x, y, z, heading]
  String stringpose = esphttp.gethttpresponse(ipaddress, requestername);
  Serial.println(stringpose);
  if (stringpose != "Error") {
    // Delimit string by commas using strtok() - need to convert to a character array first because C
    char charpose[30];
    stringpose.toCharArray(charpose, stringpose.length() + 1);
    byte index = 0;
    char *poseitems[3];
    char *ptr = NULL;
    ptr = strtok(charpose, ",");
    while (ptr != NULL) {
      poseitems[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
    }
    // Update global pose array with strings
    for (int i = 0; i < 3; i++) {
      pose[i] = atof(poseitems[i]);
    }
  }
  else {
    Serial.println("No data yet");
  }
}

void gotoheading(int heading) {
  // Function to move to a given global heading (in degrees)
  // Takes pose estimation from Python vision system
  // Moves to new pose using PID loop
  float sensitivity =  PI / 18; // 20 degree sensitivity
  getpose();
  // Check to see if the first pose message has been read
  if (pose[0] != 0) {
    Serial.println("Starting while loop");
    // Spin motors to move to heading until correct heading reached
    while (pose[2] < (heading - sensitivity) || pose[2] > (heading + sensitivity)) {
      if (millis() - current_time > 20) {
        getpose();
  
        PID_heading = pidheading.update_pid(heading, pose[2], K_P_heading, K_I_heading, K_D_heading);
  
        Serial.println(PID_heading);
  
        motors.spinmotor(0, 0, abs(PID_heading));
        motors.spinmotor(1, 1, abs(PID_heading));
  
        current_time = millis();
      }
    }
  }
  motors.spinmotor(0, 0, 0);
  motors.spinmotor(1, 0, 0);
}

void gotowaypoint(int x, int y) {
  // Function to go to a waypoint
  // First rotate to face the waypoint in question
  // Then move towards that waypoint by monitoring the heading at time intervals
  // Heading needs to be constantly monitored as the two drive units (motor + prop) have different outputs
  float sensitivity = 30;
  getpose();
  // Check to see if the first pose message has been read 
  if (pose[0] != 0) {
    Serial.println("Starting while loop");
    // Spin motors until waypoint reached within sensitivity
    while (pose[0] < x - sensitivity || pose[0] > x + sensitivity || pose[1] < y - sensitivity || pose[1] > y + sensitivity) {
      getpose();
      float headingtarget = atan2(pose[1] - y, pose[0]  - x);
      float distarget = sqrt(sq(x - pose[0]) + sq(y - pose[1]));
      
      PID_distance = piddistance.update_pid(0, distarget, K_P_distance, K_I_distance, K_D_distance);
      PID_heading = pidheading.update_pid(headingtarget, pose[2], K_P_heading, K_I_heading, K_D_heading);
      
      motors.spinmotor(0, 0, abs(PID_distance)/2 + abs(PID_heading));
      motors.spinmotor(1, 0, abs(PID_distance)/2 - abs(PID_heading));    
    }
  }
  motors.spinmotor(0, 0, 0);
  motors.spinmotor(1, 0, 0);
}


void holdposition(int x, int y) {
  // Function to hold a single position using pose requests from python
  // Currently setup to work with pixels on whichever camera is being used to locate the robot
  float sensitivity = 10;
  getpose();

  while (pose[0] < x - sensitivity && pose[0] > x + sensitivity && pose[1] < y - sensitivity && pose[1] > y + sensitivity) {
    float thetatarget = atan2(pose[1] - y, pose[0]  - x);
    float distarget = sqrt(sq(x - pose[0]) + sq(y - pose[1]));
    if (thetatarget > 3*PI/2 && thetatarget < PI/4) {
      // If the target is in front of the balloon
      gotoheading(thetatarget);
    }
  }
}
