#ifndef MYWIFI_H
#define MYWIFI_H

#include <Arduino.h>
#include <WiFi.h>

// CHANGE ME
const char* id_token = "Addy";
const char* ssid = "Molly1";
const char* password = "8hobartave";
const char* serverUrl = "http://192.168.0.116:5000/";

bool wifi_loop() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  } else {
    Serial.println("WiFi disconnected");
    return false;
  }
  
}

void wifi_setup() {
  Serial.println("WiFi Connection Test");
  
  WiFi.mode(WIFI_STA);  // Set WiFi to station mode
  WiFi.begin(ssid, password);
  
  Serial.println("Connecting to WiFi...");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 100) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("\nFailed to connect to WiFi");
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
    while (wifi_loop() == false){
      delay(100);
    }
  }
}



#endif