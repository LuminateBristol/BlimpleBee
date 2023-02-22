/*
Title: Header File for adding Wifi to an ESP32 project
Version: v2 (updated from Lumiwifi)
Date: Aug 2022
Purpose: Provide header info (class definition) for the Wifi for an ESP32.

Limitations: This v2 code has the following limitations:
            - Currently set to setting up and checking client only

Author: Henry Hickson
*/

#ifndef espwifisetup_h
#define espwifisetup_h

#include "Arduino.h"
#include "WiFi.h"
#include "WebServer.h"

class EspWifiSetup //The class defines all functions and variables used in the library
{
  public:
    //Public Variables / Functions (those used by the user in their code):

    //1. Initialiser function (constructor)
    EspWifiSetup() {      
    }

    //2. Variables note defined in functions
    String strcom;
    IPAddress ip;
    IPAddress subnet;
    IPAddress gateway;
    
    //3. Functions
    WiFiServer* configWiFi(char* ssid, char* password, WiFiServer* server) {
      server =new WiFiServer(80);
      IPAddress ip(192, 168, 1, 184);
      IPAddress gateway(192,168,4,9);
      IPAddress subnet(255,255,255,0);

      //Wifi Access Point Setup
      Serial.println("Setting AP (Access Point)...");
      Serial.println("Please connect and enter password.");
      Serial.print("ssid: ");
      Serial.println(ssid);
      WiFi.softAP(ssid,password);
      Serial.println(WiFi.softAPIP());

      server->begin();    //We use -> when working with pointers
      return server; 
    }
    
    String wificheckclient(WiFiServer* server) {
      WiFiClient client = server->available();
      if (client.available()) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println("Connection: close");
        client.println();
        client.println("Request received");
        String c = client.readStringUntil('\r');
        
        return c;
      } 
      else {
        return "null";
      }
    }

   int wifitester(WiFiServer* server) {
     String _test = wificheckclient(server);
     Serial.println(_test);
     return 100;
   }
    
 };

#endif
