#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <ESP8266HTTPClient.h>


// Your WiFi credentials
const char* ssid = "....";
const char* password = "20002000";

// Firebase host and authentication
#define FIREBASE_HOST "your-firebase-project.firebaseio.com"
#define FIREBASE_AUTH "YOUR_FIREBASE_AUTH"
FirebaseData firebaseData;


// Replace with your server name
// const char* serverName = "http://157.245.109.105:6000/trash-management/trashbin-view";
const char* serverName = "https://28f1-197-250-224-1.ngrok-free.app/amcs";
WiFiClient wificlient;
String name= "MOTOR DATA";
String id ="motor-data-1";

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void loop() {
  HTTPClient http;
    http.begin(wificlient, serverName);
  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim(); // Remove any trailing newline or space

    int powerStatus, direction, speed;
    parseData(receivedData, powerStatus, direction, speed);
    Serial.print(powerStatus);
    Serial.print(direction);
    Serial.print(speed);

    http.addHeader("Content-Type","application/json");
    DynamicJsonDocument doc(1024);
     doc["id"] = id; // Replace with your ID generation function
     doc["direction"] = direction;
     doc["power"] = powerStatus;
     doc["speed"] = speed;
     String jsonString;
     serializeJson(doc, jsonString);
     Serial.println(F("below are doc and json string"));
     Serial.println(jsonString);
     int httpResponseCode = http.POST(jsonString);
     Serial.println("\nHTTP response code: " + String(httpResponseCode));
     http.end();



    // Send data to Firebase
    // Firebase.setInt(firebaseData, "/powerStatus", powerStatus);
    // Firebase.setInt(firebaseData, "/direction", direction);
    // Firebase.setInt(firebaseData, "/speed", speed);

    Serial.println("Data sent to Firebase");
  }
}

void parseData(String data, int &powerStatus, int &direction, int &speed) {
  int commaIndex1 = data.indexOf(',');
  int commaIndex2 = data.indexOf(',', commaIndex1 + 1);

  powerStatus = data.substring(0, commaIndex1).toInt();
  direction = data.substring(commaIndex1 + 1, commaIndex2).toInt();
  speed = data.substring(commaIndex2 + 1).toInt();
}
