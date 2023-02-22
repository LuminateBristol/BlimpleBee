/*
 * ESP 32-S2 QT-PY Big Rig motor control tests.
 * Details:
 * - Three motors
 * - Two DRV8833 chips
 * - One QT-PY ESP32-S2 microcontroller
 * 
 * Author: Henry Hickson
 * Date: August 2022
 */

#include "espwifisetup.h"
#include "WiFi.h"
#include "WebServer.h"

 // Setup wifi class
EspWifiSetup espwifi;
char* ssid = "BlimpleBee";
char* password = "123456789";
WiFiServer* server;  // We use a pointer so that we can manipulate it from within the header file containing the class
int movement_status = 0;

// Setup GPIO pins for QT-PY
const int LMotor1 = 35;  
const int LMotor2 = 37;  
const int RMotor1 = 9;
const int RMotor2 = 8;
const int DMotor1 = 18;
const int DMotor2 = 17;
int dutyCycle = 50;


// Set PWM properties using the LEDC peripheral
const int freq = 5000;
const int ledChannel1 = 0;
const int ledChannel2 = 1;
const int ledChannel3 = 2;
const int ledChannel4 = 3;
const int ledChannel5 = 4;
const int ledChannel6 = 5;
const int resolution = 8;
 
void setup(){
  Serial.begin(115200);
  
  // Wifi Setup
  server = espwifi.configWiFi(ssid, password, server);
  
  // Pinouts
  pinMode(LMotor1, OUTPUT);
  pinMode(LMotor2, OUTPUT);
  pinMode(RMotor1, OUTPUT);
  pinMode(RMotor2, OUTPUT);
  pinMode(DMotor1, OUTPUT);
  pinMode(DMotor2, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(ledChannel3, freq, resolution);
  ledcSetup(ledChannel4, freq, resolution);
  ledcSetup(ledChannel5, freq, resolution);
  ledcSetup(ledChannel6, freq, resolution);

  // Attach the PWM channel to the GPIO to be controlled
  ledcAttachPin(LMotor1, ledChannel1);
  ledcAttachPin(LMotor2, ledChannel2);
  ledcAttachPin(RMotor1, ledChannel3);
  ledcAttachPin(RMotor2, ledChannel4);
  ledcAttachPin(DMotor1, ledChannel5);
  ledcAttachPin(DMotor2, ledChannel6);
}

void loop(){
  delay(100);
  readclient();
}

void readclient() {
  // Function to read all incoming data via WiFi
    String c = espwifi.wificheckclient(server);
    Serial.println(c);
    int _i = c.indexOf("HTTP");
    
    if (c != "null") {
      if (c.indexOf("rmf") != -1) {
        String motorspeed = c.substring(9, _i-1);
        Serial.print("Right motor fwd:");
        Serial.println(motorspeed);
        spinmotor(0, 0, motorspeed.toInt());
      }
      else if (c.indexOf("rmb") != -1) {
        String motorspeed = c.substring(9, _i-1);
        Serial.print("Right motor back:");
        Serial.println(motorspeed);
        spinmotor(0, 1, motorspeed.toInt());
      }
      else if (c.indexOf("lmf") != -1) {
        String motorspeed = c.substring(9, _i-1);
        Serial.print("Left motor forward:");
        Serial.println(motorspeed);
        spinmotor(1, 0, motorspeed.toInt());
      }
      else if (c.indexOf("lmb") != -1) {
        String motorspeed = c.substring(9, _i-1);
        Serial.print("Left motor back:");
        Serial.println(motorspeed);
        spinmotor(1, 1, motorspeed.toInt());
      }
      else if (c.indexOf("mmu") != -1) {
        String motorspeed = c.substring(9, _i-1);
        Serial.print("Middle motor up:");
        Serial.println(motorspeed);
        spinmotor(2, 0, motorspeed.toInt());
      }
      else if (c.indexOf("mmd") != -1) {
        String motorspeed = c.substring(9, _i-1);
        Serial.print("Middle motor down:");
        Serial.println(motorspeed);
        spinmotor(2, 1, motorspeed.toInt());
      }
    }
    else {
      return;
    }
  }

void setallmotors(int motorspeed) {
  // Function to set the speed of all motors
  // param: motorspeed - the dutycycle of the motor from 0 to 255
  ledcWrite(ledChannel1, motorspeed);
  ledcWrite(ledChannel2, 0);
  ledcWrite(ledChannel3, motorspeed);
  ledcWrite(ledChannel4, 0);
  ledcWrite(ledChannel5, motorspeed);
  ledcWrite(ledChannel6, 0);
}

void spinmotor(int motornumber, int dir, int motorspeed) {
  // Function to set a given motor to spin in a given direction and at a given speed
  // Input: motornumber - 0 = right motor, 1 = left motor, 2 = middlemotor (up/down)
  // Input: dir - direction, 0 = fwd/up, 1 = back/down
  // Input: spinspeed - pwm from 0 to 255 for motor speed
  if (motornumber == 0) {
    if (dir == 0) {
      ledcWrite(ledChannel1, motorspeed);
      ledcWrite(ledChannel2, 0);
    }
    else {
      ledcWrite(ledChannel1, 0);
      ledcWrite(ledChannel2, motorspeed);
    }
  }
  else if (motornumber == 1) {
    if (dir == 0) {
      ledcWrite(ledChannel3, motorspeed);
      ledcWrite(ledChannel4, 0);
    }
    else {
      ledcWrite(ledChannel3, 0);
      ledcWrite(ledChannel4, motorspeed);
    }
  }
  else if (motornumber == 2) {
    if (dir == 1) {
      ledcWrite(ledChannel5, motorspeed);
      ledcWrite(ledChannel6, 0);
    }
    else {
      ledcWrite(ledChannel5, 0);
      ledcWrite(ledChannel6, motorspeed);
    }
  }
}
