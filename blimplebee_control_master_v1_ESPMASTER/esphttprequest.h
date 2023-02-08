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
  void initiatehttpwifi(const char* ssid, const char* password) {
    // Setup of WiFi connection
    // Place this function in the void setup() region of main
    // param: ssid - ssid for network i.e. network name
    // param: password - network password
    
    WiFi.begin(ssid, password); 
 
    while (WiFi.status() != WL_CONNECTED) { //Check for the connection
      delay(500);
      Serial.println("Connecting..");
    }
   
    Serial.print("Connected to the WiFi network with IP: ");
    Serial.println(WiFi.localIP());
    }

  String gethttpresponse(const char* ipaddress, String requestername) {
    // Function to send http request (not a GET request but a POST request)
    // param: ipaddress - ip address to communicate over
    // param: requestername - string with name of the requesting agent
    // return: string response from the server
    // Can be used to send requests to bottle server hosted in Python
    
    if(WiFi.status()== WL_CONNECTED){   //Check WiFi connection status
      HTTPClient http; 
   
      http.begin(ipaddress);                //Specify destination for HTTP request
      http.addHeader("Content-Type", "text/plain");             //Specify content-type header
     
      int httpResponseCode = http.POST("Http request from: " + requestername);   //Send the actual POST request
     
      if(httpResponseCode>0){
       String responsestring = http.getString();
       // Serial.println(httpResponseCode);   //Print return code
       return responsestring;
            
      } else {
        return "Error";   
      }
      
      http.end();  //Free resources    
         
      } else {
        return "Error";   
      }
  }

  
};

#endif

 
