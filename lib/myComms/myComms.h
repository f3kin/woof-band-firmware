#ifndef MYCOMMS_H
#define MYCOMMS_H

#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <AsyncUDP.h>
#include <WiFiUdp.h>
#include <time.h>
#include <Arduino.h>
#include <WiFi.h>

//***************  Determine the time ***************//
const char* ntpServer = "au.pool.ntp.org";
const char* ntpServer1 = "1.au.pool.ntp.org";
const char* ntpServer2 = "2.au.pool.ntp.org";
const long gmtOffset_sec = 3600 * 11;  // GMT+11 for AEDT
const int daylightOffset_sec = 0;  // No additional offset needed during daylight saving
char timeStringBuff[50];

bool getCurrentTimeStamp() {
  struct tm timeInfo;
  bool getTimer = false;
  for (int i = 0; i < 5; i++) {
    if (getLocalTime(&timeInfo)) {
      getTimer = true;
      break;
    } else {
      Serial.println("Failed to obtain time");
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, ntpServer1, ntpServer2);
    }
  }
  strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeInfo);
  return getTimer;
}

//***************  Generate JSON Payload ****************//
StaticJsonDocument<250> jsonDocument;
char buffer[250];

inline void create_json(uint32_t stepsAmount, bool graph, AccelData data) {
  // Clear and fill json
  jsonDocument.clear();
  jsonDocument["steps"] = stepsAmount;
  if (graph){
    jsonDocument["x"] = data.x;
    jsonDocument["y"] = data.y;
    jsonDocument["z"] = data.z;
  }
  if (id_token != nullptr && strlen(id_token) > 0) {
      jsonDocument["dog_id"] = id_token;
  } else {
      jsonDocument["dog_id"] = "unknown";  // Fallback if id_token is empty
  }
  jsonDocument["time"] = timeStringBuff;

  serializeJson(jsonDocument, buffer);
}


///***************  Send data to server ****************//

// Add time delay functionality
unsigned long lastTime = 0;
unsigned long timerDelay = 100;
uint32_t lastSteps = 0;

bool send_data_to_server(uint32_t stepsAmount, bool graph, AccelData data) {
    bool ret = false;
    
    if (((millis() - lastTime) >= timerDelay) && (stepsAmount != lastSteps || graph)){
      if (WiFi.status() == WL_CONNECTED && getCurrentTimeStamp()) {
        // Setup HTTP client
        HTTPClient http;
        WiFiClient client;
        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");

        // Create JSON payload and send
        create_json(stepsAmount, graph, data);
        Serial.print("Here is the payload: ");
        Serial.println(buffer);
        int responseCode = http.POST(buffer);
        http.end();
        // Serial.print("HTTP Response code: ");
        // Serial.println(responseCode);

        // Check the Post was successful
        if (responseCode == 200) {
          Serial.println("Post successful");
          ret = true;
        }
        else{
          Serial.println("Post failed");
        } 
        lastTime = millis();
      }
      lastSteps = stepsAmount;
    }
    return ret;
}

void ready_post() {
  HTTPClient http;
  WiFiClient client;

  // Begin the connection to the server
  http.begin(client, serverUrl);

  // Set the content type for the request
  http.addHeader("Content-Type", "application/json");

  // Create the JSON payload to indicate readiness
  String payload = "{\"status\":\"calibration_complete\"}";

  // Send the POST request
  int httpResponseCode = http.POST(payload);

  // Check the response from the server
  if (httpResponseCode > 0) {
    // Successful response
    String response = http.getString();
    Serial.println("POST Response Code: " + String(httpResponseCode));
    Serial.println("Server Response: " + response);
  } else {
    // If there is an error in the HTTP request
    Serial.println("POST Request failed, error: " + String(httpResponseCode));
  }

  // End the connection
  http.end();
}

bool started = false;

void record_post(bool start) {
  HTTPClient http;
  WiFiClient client;

  // Begin the connection to the server
  http.begin(client, serverUrl);

  // Set the content type for the request
  http.addHeader("Content-Type", "application/json");

  // Create the JSON payload to indicate readiness
  String payload;
  if(start){
    payload = "{\"status\":\"recording_started\"}";}
  else{
    payload = "{\"status\":\"recording_stopped\"}";}
  // Send the POST request
  int httpResponseCode = http.POST(payload);

  // Check the response from the server
  if (httpResponseCode > 0) {
    // Successful response
    String response = http.getString();
    Serial.println("POST Response Code: " + String(httpResponseCode));
    Serial.println("Server Response: " + response);
  } else {
    // If there is an error in the HTTP request
    Serial.println("POST Request failed, error: " + String(httpResponseCode));
  }

  // End the connection
  http.end();
  started = true;
}


#endif