#include <ESP32Servo.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// --- Ultrasonic Sensor 1 (Train Detection) ---
#define TRIG_TRAIN 14
#define ECHO_TRAIN 27

// --- Ultrasonic Sensor 2 (People Detection) ---
#define TRIG_PEOPLE 26
#define ECHO_PEOPLE 25

// --- Servo Pins ---
#define SERVO1_PIN 15
#define SERVO2_PIN 13

// --- Buzzer Pin ---
#define BUZZER_PIN 12

// --- Kill Switch Pin ---
#define KILL_SWITCH_PIN 35

// --- Detection Settings ---
#define TRAIN_DISTANCE_THRESHOLD 8     // Increased for testing
#define PEOPLE_DISTANCE_THRESHOLD 5    // Increased for testing
#define TRAIN_CONFIRM_TIME 5000        // ms (Ultrasonic confirmation)
#define LOSS_CONFIRM_TIME 5000         // ms
#define PEOPLE_CHECK_TIME 5000         // ms
#define DEPARTURE_DISTANCE_THRESHOLD 8 // Distance threshold for train departure (higher than arrival)
#define MIN_CONSECUTIVE_FAR_READINGS 5  // Number of consecutive far readings needed
#define PEOPLE_CLEAR_TIME 5000         // ms - Wait 5 seconds after people clear before opening gates
#define PEOPLE_CONFIRM_TIME 5000       // ms - Check people for 5 seconds in arrival state

// --- Buzzer Settings ---
#define BUZZER_FREQUENCY 1000          // 1 kHz tone

// --- Wi-Fi and Firebase Settings ---
#define WIFI_SSID "abc"
#define WIFI_PASSWORD "12345678"
#define FIREBASE_HOST "https://triantracking-default-rtdb.firebaseio.com"

// --- GPS Stable Settings ---
const unsigned long GPS_STABLE_DURATION = 8000; // 8 seconds GPS stable
const unsigned long GPS_MOVEMENT_DURATION = 5000; // 5 seconds GPS movement for departure

// Servo objects
Servo servo1;
Servo servo2;

// --- State Variables ---
unsigned long trainDetectStart = 0;
unsigned long trainGoneStart = 0;
unsigned long peopleClearStart = 0;  // Timer for people clear confirmation
unsigned long peopleDetectStart = 0;  // Timer for people detection in arrival state
bool trainDetected = false;
bool gatesOpen = true;
bool stuckOnPeople = false;
bool stuckOnPeopleDeparture = false; // People blocking during departure
bool trainDepartedWaitingForPeople = false; // Train departed but waiting for people to clear
bool checkingPeopleArrival = false;  // Currently checking people in arrival state
bool buzzerActive = false; // Track buzzer state
int consecutiveFarReadings = 0;  // Counter for consecutive far readings

// --- GPS tracking variables ---
float lastLatitude = 0.0;
float lastLongitude = 0.0;
unsigned long gpsStableStartTime = 0;
unsigned long gpsMovementStartTime = 0;
bool gpsWasStationary = false;

// --- Function Prototypes ---
void setupWiFi();
float getDistance(int trigPin, int echoPin);
void buzzerOn();
void buzzerOff();
void closeGates();
void openGates();
void waitPeopleCheckThenCloseGates();
void waitPeopleCheckThenOpenGates();

// New GPS functions
bool getLatestGpsFromFirebase(float &latitude, float &longitude);
bool isGpsStable(float currentLat, float currentLon);
bool isGpsMoving(float currentLat, float currentLon);
float roundFloat(float value, int places);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== ESP32 Train Gate System Debug Mode ===");

  // Initialize pins
  pinMode(TRIG_TRAIN, OUTPUT);
  pinMode(ECHO_TRAIN, INPUT);
  pinMode(TRIG_PEOPLE, OUTPUT);
  pinMode(ECHO_PEOPLE, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP); // Kill switch with internal pullup

  // Ensure trigger pins start LOW
  digitalWrite(TRIG_TRAIN, LOW);
  digitalWrite(TRIG_PEOPLE, LOW);

  Serial.println("Pin setup complete");

  // Test buzzer
  Serial.println("Testing buzzer...");
  buzzerOn();
  delay(500);
  buzzerOff();
  Serial.println("Buzzer test complete");

  // Initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Ensure gates start OPEN
  Serial.println("Setting gates to OPEN position...");
  servo1.write(0);    // Gate 1 open position
  servo2.write(180);  // Gate 2 open position  
  gatesOpen = true;
  delay(2000); // Give servos time to reach position

  Serial.println("Servos initialized - Gates OPEN");

  // --- Wi-Fi Setup ---
  setupWiFi();
  // ------------------------------------
}

void setupWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  float distance = (duration * 0.034) / 2;

  if (distance < 1 || distance > 400) {
    return -1;
  }

  return distance;
}

// --- Continuous buzzer tone like your test code ---
void buzzerOn() {
  if (!buzzerActive) {
    tone(BUZZER_PIN, BUZZER_FREQUENCY); // Play tone forever
    buzzerActive = true;
    Serial.println("BUZZER ON - People detected during train arrival");
  }
}

void buzzerOff() {
  if (buzzerActive) {
    noTone(BUZZER_PIN); // Stop tone
    buzzerActive = false;
    Serial.println("BUZZER OFF - No people detected");
  }
}

float getTrainDistance() {
  return getDistance(TRIG_TRAIN, ECHO_TRAIN);
}

float getPeopleDistance() {
  return getDistance(TRIG_PEOPLE, ECHO_PEOPLE);
}

void closeGates() {
  if (gatesOpen) {
    Serial.println("Closing gates smoothly...");
    
    // Smooth servo movement - close gradually
    for (int pos1 = 0, pos2 = 180; pos1 <= 90 && pos2 >= 90; pos1++, pos2--) {
      servo1.write(pos1);
      servo2.write(pos2);
      delay(20); // Smooth movement delay
    }
    
    gatesOpen = false;
    Serial.println("Gates CLOSED");
  }
}

void openGates() {
  if (!gatesOpen) {
    Serial.println("Opening gates smoothly...");
    
    // Smooth servo movement - open gradually
    for (int pos1 = 90, pos2 = 90; pos1 >= 0 && pos2 <= 180; pos1--, pos2++) {
      servo1.write(pos1);
      servo2.write(pos2);
      delay(20); // Smooth movement delay
    }
    
    gatesOpen = true;
    Serial.println("Gates OPEN");
  }
}

void waitPeopleCheckThenCloseGates() {
  Serial.println("Checking for people before closing gates...");
  unsigned long startTime = millis();
  bool peopleFound = false;

  while (millis() - startTime < PEOPLE_CHECK_TIME) {
    float peopleDist = getPeopleDistance();

    if (peopleDist > 0) {
      Serial.print("People distance: ");
      Serial.print(peopleDist);
      Serial.println(" cm");

      if (peopleDist <= PEOPLE_DISTANCE_THRESHOLD) {
        Serial.println("PEOPLE DETECTED - Cannot close gates!");
        stuckOnPeople = true;
        peopleFound = true;
        break;
      }
    }
    delay(200);
  }

  if (!peopleFound) {
    stuckOnPeople = false;
    Serial.println("Area clear - Safe to close gates");
    closeGates();
  }
}

void waitPeopleCheckThenOpenGates() {
  Serial.println("Checking for people before opening gates...");
  unsigned long startTime = millis();
  bool peopleFound = false;

  while (millis() - startTime < PEOPLE_CHECK_TIME) {
    float peopleDist = getPeopleDistance();

    if (peopleDist > 0) {
      Serial.print("People distance: ");
      Serial.print(peopleDist);
      Serial.println(" cm");

      if (peopleDist <= PEOPLE_DISTANCE_THRESHOLD) {
        Serial.println("PEOPLE DETECTED - Cannot open gates!");
        stuckOnPeople = true;
        peopleFound = true;
        break;
      }
    }
    delay(200);
  }

  if (!peopleFound) {
    stuckOnPeople = false;
    Serial.println("Area clear - Safe to open gates");
    openGates();
  }
}

// Helper function to round a float to a specified number of decimal places
float roundFloat(float value, int places) {
    float multiplier = pow(10, places);
    return round(value * multiplier) / multiplier;
}

// New function to fetch the latest GPS data from Firebase
bool getLatestGpsFromFirebase(float &latitude, float &longitude) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return false;
  }

  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/gps_locations.json?orderBy=\"$key\"&limitToLast=1";
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        JsonObject obj = doc.as<JsonObject>();
        if (obj.size() > 0) {
          auto it = obj.begin();
          JsonObject latestLocation = it->value().as<JsonObject>();
          latitude = latestLocation["latitude"] | 0.0;
          longitude = latestLocation["longitude"] | 0.0;
          http.end();
          return true;
        } else {
          Serial.println("No location data found");
        }
      } else {
        Serial.println("JSON parse failed");
      }
    } else {
      Serial.printf("HTTP error: %d\n", httpCode);
    }
  } else {
    Serial.printf("GET request failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  return false;
}

// Function to check if GPS is stable (for train arrival)
bool isGpsStable(float currentLat, float currentLon) {
  const int decimalPlaces = 4;
  float roundedCurrentLat = roundFloat(currentLat, decimalPlaces);
  float roundedCurrentLon = roundFloat(currentLon, decimalPlaces);
  float roundedLastLat = roundFloat(lastLatitude, decimalPlaces);
  float roundedLastLon = roundFloat(lastLongitude, decimalPlaces);

  bool closeEnough = (roundedCurrentLat == roundedLastLat) && (roundedCurrentLon == roundedLastLon);

  if (closeEnough) {
    if (gpsStableStartTime == 0) {
      gpsStableStartTime = millis();
      Serial.println("GPS location is stable. Starting timer...");
    } else if (millis() - gpsStableStartTime >= GPS_STABLE_DURATION) {
      gpsWasStationary = true;
      return true;
    }
  } else {
    gpsStableStartTime = 0;
    gpsWasStationary = false;
    lastLatitude = currentLat;
    lastLongitude = currentLon;
    Serial.println("GPS location has moved. Resetting stability timer.");
  }
  return false;
}

// Function to check if GPS is moving (for train departure)
bool isGpsMoving(float currentLat, float currentLon) {
  const int decimalPlaces = 4;
  float roundedCurrentLat = roundFloat(currentLat, decimalPlaces);
  float roundedCurrentLon = roundFloat(currentLon, decimalPlaces);
  float roundedLastLat = roundFloat(lastLatitude, decimalPlaces);
  float roundedLastLon = roundFloat(lastLongitude, decimalPlaces);

  bool hasMovedSignificantly = (roundedCurrentLat != roundedLastLat) || (roundedCurrentLon != roundedLastLon);

  if (hasMovedSignificantly) {
    if (gpsMovementStartTime == 0) {
      gpsMovementStartTime = millis();
      Serial.println("GPS movement detected. Starting movement timer...");
    } else if (millis() - gpsMovementStartTime >= GPS_MOVEMENT_DURATION) {
      return true;
    }
  } else {
    gpsMovementStartTime = 0;
    Serial.println("GPS movement stopped. Resetting movement timer.");
  }
  
  return false;
}

void loop() {
  // Check kill switch first - if HIGH (ON), freeze the system
  if (digitalRead(KILL_SWITCH_PIN) == LOW) { // Switch is ON (pulled to ground)
    Serial.println(">>> KILL SWITCH ACTIVATED - SYSTEM FROZEN <<<");
    buzzerOff(); // Turn off buzzer when system is frozen
    
    // Freeze loop - just wait and check kill switch periodically
    while (digitalRead(KILL_SWITCH_PIN) == LOW) {
      Serial.println("System frozen by kill switch...");
      delay(1000); // Check every second
    }
    
    Serial.println(">>> KILL SWITCH DEACTIVATED - RESUMING AUTOMATED SYSTEM <<<");
    delay(500); // Small delay to avoid switch bounce
  }
  
  // Handle people stuck scenarios during departure
  if (stuckOnPeopleDeparture) {
    // DEPARTURE STAGE: People present - buzzer OFF, continue train detection
    float peopleDist = getPeopleDistance();
    Serial.print("DEPARTURE - People check: ");
    if (peopleDist == -1) {
      Serial.println("No reading");
    } else {
      Serial.print(peopleDist);
      Serial.println(" cm");
    }
    
    if (peopleDist == -1 || peopleDist > PEOPLE_DISTANCE_THRESHOLD) {
      if (peopleClearStart == 0) {
        peopleClearStart = millis();
        Serial.println("People clearing detected - Starting 5 second confirmation...");
      } else {
        unsigned long elapsed = millis() - peopleClearStart;
        if (elapsed >= PEOPLE_CLEAR_TIME) {
          Serial.println("People cleared during departure - Continuing train detection");
          stuckOnPeopleDeparture = false;
          peopleClearStart = 0;
        } else {
          Serial.print("Confirming people clear... ");
          Serial.print(elapsed / 1000.0, 1);
          Serial.println("/5.0 seconds");
        }
      }
    } else {
      // People still detected, reset timer
      peopleClearStart = 0;
    }
    // Continue with normal train detection logic below
  }
  
  // Handle case where train has departed but waiting for people to clear
  if (trainDepartedWaitingForPeople) {
    float peopleDist = getPeopleDistance();
    Serial.print("TRAIN DEPARTED - Waiting for people to clear: ");
    if (peopleDist == -1) {
      Serial.println("No reading");
    } else {
      Serial.print(peopleDist);
      Serial.println(" cm");
    }
    
    if (peopleDist == -1 || peopleDist > PEOPLE_DISTANCE_THRESHOLD) {
      if (peopleClearStart == 0) {
        peopleClearStart = millis();
        Serial.println("People clearing detected - Starting 5 second confirmation before opening gates...");
      } else {
        unsigned long elapsed = millis() - peopleClearStart;
        if (elapsed >= PEOPLE_CLEAR_TIME) {
          Serial.println(">>> PEOPLE CLEARED - OPENING GATES <<<");
          waitPeopleCheckThenOpenGates();
          trainDepartedWaitingForPeople = false;
          stuckOnPeopleDeparture = false;
          peopleClearStart = 0;
          // Reset GPS timers
          gpsStableStartTime = 0;
          gpsMovementStartTime = 0;
          gpsWasStationary = false;
        } else {
          Serial.print("Confirming people clear before opening gates... ");
          Serial.print(elapsed / 1000.0, 1);
          Serial.println("/5.0 seconds");
        }
      }
    } else {
      // People still detected, reset timer
      peopleClearStart = 0;
    }
    delay(1000);
    return; // Don't do train detection while waiting for people to clear after departure
  }

  // Handle general people stuck scenario (both arrival and departure)
  if (stuckOnPeople) {
    float peopleDist = getPeopleDistance();
    Serial.print("PEOPLE STUCK - Waiting for people to clear: ");
    if (peopleDist == -1) {
      Serial.println("No reading");
    } else {
      Serial.print(peopleDist);
      Serial.println(" cm");
    }
    
    if (peopleDist == -1 || peopleDist > PEOPLE_DISTANCE_THRESHOLD) {
      if (peopleClearStart == 0) {
        peopleClearStart = millis();
        Serial.println("People clearing detected - Starting 5 second confirmation...");
      } else {
        unsigned long elapsed = millis() - peopleClearStart;
        if (elapsed >= PEOPLE_CLEAR_TIME) {
          Serial.println(">>> PEOPLE CLEARED - RESUMING NORMAL OPERATION <<<");
          stuckOnPeople = false;
          peopleClearStart = 0;
          // Continue with normal train detection
        } else {
          Serial.print("Confirming people clear... ");
          Serial.print(elapsed / 1000.0, 1);
          Serial.println("/5.0 seconds");
        }
      }
    } else {
      // People still detected, reset timer
      peopleClearStart = 0;
    }
    delay(1000);
    return; // Don't continue with train detection while people are stuck
  }

  float trainDist = -1;
  for (int attempt = 3; attempt > 0; attempt--) {
    trainDist = getTrainDistance();
    if (trainDist > 0) break;
    delay(50);
  }

  // --- GPS Logic (only for arrival detection) ---
  float currentLat = 0.0;
  float currentLon = 0.0;
  bool gpsValid = getLatestGpsFromFirebase(currentLat, currentLon);
  bool isGpsStationary = false;
  
  if (gpsValid && !trainDetected) {
    // Only check GPS stability for arrival detection
    isGpsStationary = isGpsStable(currentLat, currentLon);
  }
  // ----------------------

  Serial.print("Train Distance: ");
  if (trainDist == -1) {
    Serial.print("No reading");
  } else {
    Serial.print(trainDist);
    Serial.print(" cm");
  }
  Serial.print(" | GPS Status: ");
  if (!trainDetected) {
    Serial.print(isGpsStationary ? "STATIONARY" : "NOT_STATIONARY");
  } else {
    Serial.print("N/A (departure mode)");
  }
  Serial.print(" | Train Detected: ");
  Serial.print(trainDetected ? "YES" : "NO");
  Serial.print(" | Gates: ");
  Serial.println(gatesOpen ? "OPEN" : "CLOSED");

  unsigned long currentMillis = millis();

  if (!trainDetected) {
    // TRAIN ARRIVAL DETECTION
    
    // Check for people during arrival - with buzzer control
    float peopleDist = getPeopleDistance();
    bool peoplePresent = (peopleDist > 0 && peopleDist <= PEOPLE_DISTANCE_THRESHOLD);
    
    if (!checkingPeopleArrival && (trainDist > 0 && trainDist <= TRAIN_DISTANCE_THRESHOLD) && isGpsStationary) {
      // Start people checking process
      checkingPeopleArrival = true;
      peopleDetectStart = currentMillis;
      Serial.println(">>> TRAIN ARRIVAL CONDITIONS MET - STARTING 5 SECOND PEOPLE CHECK <<<");
    }
    
    if (checkingPeopleArrival) {
      unsigned long peopleElapsed = currentMillis - peopleDetectStart;
      
      if (peoplePresent) {
        // Turn buzzer ON when people detected during arrival check
        buzzerOn();
        
        Serial.print("PEOPLE CHECK - People detected: ");
        Serial.print(peopleDist);
        Serial.print(" cm | Time: ");
        Serial.print(peopleElapsed / 1000.0, 1);
        Serial.println("/5.0 seconds");
        
        if (peopleElapsed >= PEOPLE_CONFIRM_TIME) {
          // People confirmed for 5 seconds - Wait for people to clear
          Serial.println(">>> PEOPLE CONFIRMED FOR 5 SECONDS - WAITING FOR PEOPLE TO CLEAR <<<");
          Serial.println(">>> BUZZER ON - CONTINUING TRAIN DETECTION <<<");
          stuckOnPeople = true; // Use general stuck flag
          checkingPeopleArrival = false;
          trainDetectStart = 0;
          return;
        }
      } else {
        // Turn buzzer OFF when no people detected during arrival check
        buzzerOff();
        
        Serial.print("PEOPLE CHECK - Area clear | Time: ");
        Serial.print(peopleElapsed / 1000.0, 1);
        Serial.println("/5.0 seconds");
        
        if (peopleElapsed >= PEOPLE_CONFIRM_TIME) {
          // No people for 5 seconds - ALLOW TRAIN DETECTION TO PROCEED
          Serial.println(">>> NO PEOPLE FOR 5 SECONDS - PROCEEDING WITH TRAIN ARRIVAL <<<");
          checkingPeopleArrival = false;
          
          // Now proceed with normal train detection
          if (trainDetectStart == 0) {
            trainDetectStart = currentMillis;
            Serial.println(">>> TRAIN DETECTION STARTED (ULTRASONIC + GPS STATIONARY) <<<");
          }
          
          unsigned long elapsed = currentMillis - trainDetectStart;
          if (elapsed >= TRAIN_CONFIRM_TIME) {
            trainDetected = true;
            Serial.println(">>> TRAIN CONFIRMED - CLOSING GATES <<<");
            closeGates();
            trainGoneStart = 0;
          } else {
            Serial.print("Confirming train arrival... ");
            Serial.print(elapsed / 1000.0, 1);
            Serial.println("/5.0 seconds");
          }
        }
      }
    } else if (!checkingPeopleArrival) {
      // Reset detection if conditions not met and ensure buzzer is off
      trainDetectStart = 0;
      buzzerOff();
    }
  } else {
    // TRAIN DEPARTURE DETECTION - Only using ultrasonic sensor
    // Ensure buzzer is off during departure phase
    buzzerOff();
    
    // Check for people during departure stage (different behavior)
    float peopleDist = getPeopleDistance();
    if (peopleDist > 0 && peopleDist <= PEOPLE_DISTANCE_THRESHOLD) {
      if (!stuckOnPeopleDeparture) {
        Serial.println("PEOPLE DETECTED DURING DEPARTURE - Waiting for people to clear (no buzzer)");
        stuckOnPeopleDeparture = true;
      }
      // Continue with train detection even with people present during departure
    }
    
    // Check if ultrasonic shows train is far away or no reading
    bool ultrasonicClear = (trainDist == -1) || (trainDist > DEPARTURE_DISTANCE_THRESHOLD);
    
    if (ultrasonicClear) {
      // Count consecutive far/no readings
      consecutiveFarReadings++;
      
      if (trainGoneStart == 0) {
        trainGoneStart = currentMillis;
        Serial.println(">>> TRAIN DEPARTURE DETECTED (ULTRASONIC CLEAR) <<<");
        Serial.print("Consecutive clear readings: ");
        Serial.println(consecutiveFarReadings);
      }

      unsigned long elapsed = currentMillis - trainGoneStart;
      if (elapsed >= LOSS_CONFIRM_TIME && consecutiveFarReadings >= MIN_CONSECUTIVE_FAR_READINGS) {
        // Train has departed - check for people before opening gates
        if (!stuckOnPeopleDeparture) {
          // No people detected, safe to open gates immediately
          trainDetected = false;
          consecutiveFarReadings = 0;
          trainGoneStart = 0; // Reset departure timer
          Serial.println(">>> TRAIN DEPARTED - OPENING GATES <<<");
          waitPeopleCheckThenOpenGates();
          // Reset GPS timers
          gpsStableStartTime = 0;
          gpsMovementStartTime = 0;
          gpsWasStationary = false;
        } else {
          // People are present, set flag to wait for people to clear
          trainDetected = false;
          consecutiveFarReadings = 0;
          trainGoneStart = 0; // Reset departure timer
          trainDepartedWaitingForPeople = true;
          peopleClearStart = 0; // Reset people clear timer
          Serial.println(">>> TRAIN DEPARTED BUT PEOPLE PRESENT - WAITING FOR PEOPLE TO CLEAR <<<");
        }
      } else {
        Serial.print("Confirming train departure... ");
        Serial.print(elapsed / 1000.0, 1);
        Serial.print("/5.0 seconds | Clear readings: ");
        Serial.print(consecutiveFarReadings);
        Serial.print("/");
        Serial.print(MIN_CONSECUTIVE_FAR_READINGS);
        if (stuckOnPeopleDeparture) {
          Serial.print(" | PEOPLE PRESENT");
        }
        Serial.println();
      }
    } else {
      // Train still detected close
      // FIXED: DO NOT reset departure detection when people are present
      // Only reset if train is actually still close (not due to people)
      if (!stuckOnPeopleDeparture) {
        // Only reset if no people issue - train is genuinely still close
        trainGoneStart = 0;
        consecutiveFarReadings = 0;
        Serial.print("Departure reset: Train still close (");
        Serial.print(trainDist);
        Serial.println(" cm)");
      } else {
        // People present but don't reset departure detection
        // Just don't increment consecutive readings
        consecutiveFarReadings = 0; // Reset consecutive count but keep timer
        Serial.print("Train departure continuing despite people present (train: ");
        Serial.print(trainDist);
        Serial.println(" cm)");
      }
    }
  }

  delay(300);
}