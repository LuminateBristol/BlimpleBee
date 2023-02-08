/*
Title: Header File for adding http requests to an ESP32 project
Version: v1
Date: Sept 2022
Author: Henry Hickson

Notes:
- The network credentials for the connected WiFi network myst be entered
- The IP address over which http communication takes place must be entered
- The main function is the gethttpresponse which is used to contact the server and return the String response
- This works well with a Python server such as the bottle server
- Note this is not technically a GET request
*/

#ifndef esphttprequest_h
#define esphttprequest_h

#include "WiFi.h"
#include "HTTPClient.h"

class EspHttpRequest {
  public:

  // 1. Constructor
  EspHttpRequest() {
  }

  // 2. Variables

  // 3. Functions
  void initiatehttpwifi(char* ssid, char* password) {
    // Setup of WiFi connection
    // Place this function in the void setup() region of main
    WiFi.begin(ssid, password); 
 
    while (WiFi.status() != WL_CONNECTED) { //Check for the connection
      delay(500);
      Serial.println("Connecting..");
    }
   
    Serial.print("Connected to the WiFi network with IP: ");
    Serial.println(WiFi.localIP());
    }

  String gethttpresponse() {
    if(WiFi.status()== WL_CONNECTED){   //Check WiFi connecti A\on status
      HTTPClient http;   
   
      http.begin("http://192.168.43.136:8090/");                //Specify destination for HTTP request
      http.addHeader("Content-Type", "text/plain");             //Specify content-type header
     
      int httpResponseCode = http.POST("Hello Dickhead, from ESP32");   //Send the actual POST request
     
      if(httpResponseCode>0){
       String responsestring = http.getString();
       // Serial.println(httpResponseCode);   //Print return code
       return responsestring;     
      } else {
        return "Error on sending request: ";r     
      }
      
      http.end();  //Free resources    
         
      } else {
        return "Error in WiFi connection";   
      }
  }

  
};

#endif

 
