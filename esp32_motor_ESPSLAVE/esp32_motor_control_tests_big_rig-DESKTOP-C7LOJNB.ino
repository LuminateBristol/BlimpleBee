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
WebServer* server;  // We use a pointer so that we can manipulate it from within the header file containing the class
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
  server->on("/", handle_OnConnect);
  server->on("/stop", handle_stop);
  server->on("/fwd", handle_fwd);
  server->on("/spin", handle_spin);
  server->on("/up", handle_up);
  server->on("/down", handle_down);
  
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
  server->handleClient();
  server->send(200, "text/html", SendHTML(movement_status));
  Serial.println("working");
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

void flyforward(int fwdspeed) {
  // Function to set the forward flying speed of the outboard motors
  // param: fwd speed - the duty cycle of the motors from 0 to 255
  ledcWrite(ledChannel1, fwdspeed);
  ledcWrite(ledChannel2, 0);
  ledcWrite(ledChannel3, fwdspeed);
  ledcWrite(ledChannel4, 0);
}

void spinonspot(bool dir, int spinspeed) {
  // Function to set the spin on spot speed flying speed of the outboard motors
  // param: dir - the direction to spin in - boolean 1 or 0
  // param: spinspeed - the duty cycle of the motors from 0 to 255
  if (dir) {
    ledcWrite(ledChannel1, spinspeed);
    ledcWrite(ledChannel2, 0);
    ledcWrite(ledChannel3, 0);
    ledcWrite(ledChannel4, spinspeed);
  }
  else {
    ledcWrite(ledChannel1, 0);
    ledcWrite(ledChannel2, spinspeed);
    ledcWrite(ledChannel3, spinspeed);
    ledcWrite(ledChannel4, 0);
  }
}

void changeheight(bool dir, int heightspeed) {
  // Function to set the speed of the downward facing motor
  // param: heightspeed - the duty cycle of the motors from 0 to 255
  if (dir) {
    ledcWrite(ledChannel5, heightspeed);
    ledcWrite(ledChannel6, 0);
  }
  else {
    ledcWrite(ledChannel5, 0);
    ledcWrite(ledChannel6, heightspeed);
  }
}

void handle_OnConnect() {
  setallmotors(0);
  Serial.println("Connection received");
}

void handle_stop() {
  setallmotors(0);
  movement_status = 0;
}

void handle_fwd() {
  flyforward(150);
  movement_status = 1;
}        

void handle_spin() {
  spinonspot(1, 150);
}

void handle_up() {
  changeheight(0, 150);
}        

void handle_down() {
  changeheight(0, 150);
}        

String SendHTML(uint8_t fwdstat) {
  String ptr = "<!DOCTYPE html><html>\n>";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>BlimpleBee Control</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>BlimpleBee Control</h1>\n";
  ptr +="<h3>WiFi Control Centre</h3>\n";

  if(fwdstat == 1)
    {ptr +="<p>Move fwd: ON </p><a class=\"button button-off\" href=\"/stop\">OFF</a>\n";}
  else
    {ptr +="<p>Move fwd: OFF </p><a class=\"button button-on\" href=\"/fwd\">ON</a>\n";}


  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}
