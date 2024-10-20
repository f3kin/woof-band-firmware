# const char* ssid = "Aussie Broadband 5699";
# const char* password = "Nahfyscfnu";

const char* ssid = "Molly1";
const char* password = "8hobartave";

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give the serial connection time to start

  Serial.println("WiFi Connection Test");
  
  WiFi.mode(WIFI_STA);  // Set WiFi to station mode
  WiFi.begin(ssid, password);
  
  Serial.println("Connecting to WiFi...");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
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
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi still connected");
  } else {
    Serial.println("WiFi disconnected");
  }
  delay(5000);  // Check every 5 seconds
}
