#include <WiFi.h>
#include <FirebaseESP32.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <time.h>

// WiFi credentials
#define WIFI_SSID "abc"
#define WIFI_PASSWORD "12345678"

// Firebase configuration
#define FIREBASE_HOST "triantracking-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "LM2WM02aL2vUSq1aAFPUcDAkLtPTYXqUUsGHX69J"

// GPS module pins (ESP32 has multiple hardware serial ports)
#define GPS_RX_PIN 16  // Connect to GPS TX
#define GPS_TX_PIN 17  // Connect to GPS RX

// Create instances
HardwareSerial gpsSerial(2);  // Use UART2 on ESP32
TinyGPSPlus gps;
FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

// Variables
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 2000; // Send data every 10 seconds

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // Initialize UART2 with custom pins
  
  Serial.println("Starting GPS to Firebase system on ESP32...");
  
  // Initialize WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Initialize Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  // Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  
  // Set the size of WiFi rx/tx buffers
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
  
  Serial.println("Firebase initialized!");
  Serial.println("Waiting for GPS signal...");
  Serial.println("ESP32 UART2 configured for GPS communication");
}

void loop() {
  // Read GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Check if we have valid location data
      if (gps.location.isValid()) {
        // Check if it's time to send data
        if (millis() - lastUpdate >= updateInterval) {
          sendLocationToFirebase();
          lastUpdate = millis();
        }
      }
    }
  }
  
  // Check if GPS is not receiving data
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected: check wiring.");
    delay(5000);
  }
}

void sendLocationToFirebase() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Sending location data to Firebase...");
    
    FirebaseJson json;
    
    // Add GPS data
    json.set("latitude", gps.location.lat());
    json.set("longitude", gps.location.lng());
    json.set("altitude", gps.altitude.meters());
    json.set("speed", gps.speed.kmph());
    json.set("satellites", gps.satellites.value());
    json.set("hdop", gps.hdop.hdop());
    
    // Add timestamp
    json.set("timestamp", millis());
    
    // Add date and time if available
    if (gps.date.isValid()) {
      char dateStr[20];
      sprintf(dateStr, "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
      json.set("date", dateStr);
    }
    
    if (gps.time.isValid()) {
      char timeStr[20];
      sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      json.set("time", timeStr);
    }
    
    // Use fixed path for location updates
    const char* path = "/gps_locations/location";
    
    if (Firebase.setJSON(firebaseData, path, json)) {
      Serial.println("Data updated successfully at:");
      Serial.println(path);
      Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Failed to update data");
      Serial.println("Reason: " + firebaseData.errorReason());
    }
    
    Serial.println("--------------------");
  } else {
    Serial.println("WiFi not connected!");
  }
}

void printGPSInfo() {
  Serial.print("Location: ");
  if (gps.location.isValid()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", Lng: ");
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("INVALID");
  }

  Serial.print("  Date/Time: ");
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.year());
  } else {
    Serial.print("INVALID");
  }

  Serial.print(" ");
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print("0");
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print("0");
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print("0");
    Serial.print(gps.time.second());
  } else {
    Serial.print("INVALID");
  }

  Serial.println();
}
