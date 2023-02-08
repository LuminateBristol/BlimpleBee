/*
Title: Header File for BlimpleBee motor control
Version: v1
Date: Sept 2022
Author: Henry Hickson
*/

#ifndef motorcontrol_h
#define motorcontrol_h

class MotorControl {
  public:

  // 1. Constructor
  MotorControl() {
  }

  // 2. Variables
  // Setup GPIO pins for QT-PY
  const int LMotor1 = 35;  
  const int LMotor2 = 37;  
  const int RMotor1 = 8;
  const int RMotor2 = 9;
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
  

  // 3. Functions
  void motorsetup() {
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
    if (dir == 0) {
      ledcWrite(ledChannel5, motorspeed);
      ledcWrite(ledChannel6, 0);
    }
    else {
      ledcWrite(ledChannel5, 0);
      ledcWrite(ledChannel6, motorspeed);
    }
  }
  }
  
};

#endif

 
