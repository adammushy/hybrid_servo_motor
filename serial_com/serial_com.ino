#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
// #include <UUID.h> 
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>


/* 2. Define the API Key */
#define API_KEY "AIzaSyB7s-Lzzk6IEV6XC3Y6Ojte4hTMGV7JpcM"

/* 3. Define the project ID */
#define FIREBASE_PROJECT_ID "smart-garbage-collection-8fb03"

#define USER_EMAIL "esp8266@gmail.com"
#define USER_PASSWORD "123456"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

long dataMillis = 0;
bool taskcomplete = false;
String message;

// TinyGPSPlus gps;  // The TinyGPS++ object

// SoftwareSerial Serial2(2, 15); // The serial connection to the GPS device

const char* ssid = "....";
const char* password = "20002000";


void setup(){
    Serial.begin(9600);
    WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
    /* Assign the api key (required) */
    config.api_key = API_KEY;
    /* Assign the user sign in credentials */
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback; 
  // Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  #if defined(ESP8266)
    // In ESP8266 required for BearSSL rx/tx buffer for large data handle, increase Rx size as needed.
    fbdo.setBSSLBufferSize(2048 /* Rx buffer size in bytes from 512 - 16384 */, 2048 /* Tx buffer size in bytes from 512 - 16384 */);
#endif
    fbdo.setResponseSize(2048);

    Firebase.begin(&config, &auth);

    Firebase.reconnectWiFi(true);

  delay(500);


}

void loop(){
  Serial.println("hellow from ESP8266");

  // if serial avalable
  if(Serial.available()){
    message = Serial.readString();
    Serial.println("Received: " + message);
    delay(1000);
  }

  // Firestore operations (if needed)
  // Note: Make sure to include and configure the Firebase library properly
  if (Firebase.ready() && (millis() - dataMillis > 15000 || dataMillis == 0)) {
    dataMillis = millis();
    FirebaseJson content;
    String documentPath = "data/"+String("motordata");
    if (!taskcomplete) {
      taskcomplete = true;
      content.clear();
      content.set("fields/Latitude/doubleValue", 20.0000);
      content.set("fields/Longitude/doubleValue", 19.0000);
      content.set("fields/state/stringValue", message);
      content.set("fields/percentage/integerValue", 76);
      content.set("fields/name/stringValue", "motor");
      Serial.print("Create a document... ");
      if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw()))
        Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
      else
        Serial.println(fbdo.errorReason());
    }
    content.clear();
      content.set("fields/Latitude/doubleValue", 20.0000);
      content.set("fields/Longitude/doubleValue", 19.0000);
      content.set("fields/state/stringValue", message);
      content.set("fields/percentage/integerValue", 76);
      content.set("fields/name/stringValue", "motor");
    Serial.print("Update a Smart... ");
    if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw(), "name,state,Latitude,Longitude,percentage" /* updateMask */))
      Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
    else
        Serial.println(fbdo.errorReason());
  }
  delay(1000);
}


