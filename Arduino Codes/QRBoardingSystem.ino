#include <WiFi.h>
#include <time.h>
#include <ESP32QRCodeReader.h>
#include <ESP32Servo.h>

// -------------------- WiFi credentials --------------------
const char* ssid     = "abc";
const char* password = "12345678";

// -------------------- NTP Server --------------------
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800; // Sri Lanka GMT +5:30
const int   daylightOffset_sec = 0;

// -------------------- QR Code Reader --------------------
ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);

// -------------------- Servo --------------------
Servo gateServo;
#define SERVO_PIN 15
#define OPEN_ANGLE 90
#define CLOSE_ANGLE 0
#define SERVO_SPEED 10  // smaller = slower (ms delay per degree)

// -------------------- Ultrasonic Sensor --------------------
#define TRIG_PIN  12
#define ECHO_PIN  13
#define DETECT_DISTANCE_CM 4   // Person detection threshold
#define LEAVE_DELAY_MS 4000    // Wait 4 seconds after leaving before closing

// -------------------- LED --------------------
#define GREEN_LED_PIN 2  // LED lights up when scan is successful

// -------------------- QR & Scan Lock --------------------
String lastScannedPayload = "";       // last scanned QR
unsigned long qrLockTime = 0;         // time QR was scanned
const unsigned long qrLockDuration = 5000; // 5-second QR lock
unsigned long lastQrScanTime = 0;     // last QR scan attempt
const unsigned long qrScanInterval = 1000; // scan every 1 second

// -------------------- Gate Cycle Lock --------------------
bool gateInCycle = false; // true while gate is opening/closing

// -------------------- Smooth Servo Functions --------------------
void smoothMoveServo(int startAngle, int endAngle) {
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos++) {
      gateServo.write(pos);
      delay(SERVO_SPEED);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos--) {
      gateServo.write(pos);
      delay(SERVO_SPEED);
    }
  }
}

void openGateSmoothly() {
  Serial.println("Opening gate smoothly...");
  smoothMoveServo(CLOSE_ANGLE, OPEN_ANGLE);
}

void closeGateSmoothly() {
  Serial.println("Closing gate smoothly...");
  smoothMoveServo(OPEN_ANGLE, CLOSE_ANGLE);
}

// -------------------- WiFi + Time Setup --------------------
void setupWiFiAndTime() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println("Time synchronized");
}

// -------------------- Get Current Date --------------------
String getCurrentDate() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "";
  }
  char dateStr[11];
  strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &timeinfo);
  return String(dateStr);
}

// -------------------- Ultrasonic --------------------
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.0343 / 2;
  return distance;
}

// -------------------- QR Flush Helper --------------------
void flushQrBuffer() {
  Serial.println("Clearing QR camera buffer...");
  struct QRCodeData tempQr;
  unsigned long start = millis();
  while (millis() - start < 1000) {  // flush for 1 second
    reader.receiveQrCode(&tempQr, 50);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  Serial.println("QR buffer cleared");
}

// -------------------- QR Task --------------------
void onQrCodeTask(void *pvParameters) {
  struct QRCodeData qrCodeData;

  while (true) {
    float distance = readUltrasonicDistance();

    // If gate is in cycle (opened), handle ultrasonic-based close logic
    if (gateInCycle) {
      static bool personDetected = false;
      static unsigned long leaveStartTime = 0;

      // If person detected (≤ DETECT_DISTANCE_CM)
      if (distance > 0 && distance <= DETECT_DISTANCE_CM) {
        if (!personDetected) {
          Serial.println("Person detected inside gate area");
          personDetected = true;
        }
        // Person is present, reset any leave timer
        leaveStartTime = 0;
      }
      // Person not detected (> DETECT_DISTANCE_CM)
      else {
        if (personDetected && leaveStartTime == 0) {
          leaveStartTime = millis();
          Serial.println("Person left area - starting 4s countdown...");
        }

        // If person doesn't return within 4 seconds, close gate
        if (leaveStartTime > 0 && (millis() - leaveStartTime >= LEAVE_DELAY_MS)) {
          Serial.println("No person for 4 seconds - closing gate...");
          closeGateSmoothly();
          gateInCycle = false;
          personDetected = false;
          leaveStartTime = 0;

          // --- Add 4-second cooldown after gate closes ---
          Serial.println("Cooling down for 4 seconds before next scan...");
          vTaskDelay(4000 / portTICK_PERIOD_MS);  // pause all scanning

          // --- Clear any old QR from camera memory ---
          flushQrBuffer();

          // Turn off green LED after buffer is cleared and ready for next scan
          digitalWrite(GREEN_LED_PIN, LOW);

          lastScannedPayload = ""; // allow next QR
          Serial.println("Gate closed - ready for next QR scan");
        }
      }

      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue; // Skip QR scanning during gate cycle
    }

    // Throttle QR scanning
    if (millis() - lastQrScanTime < qrScanInterval) {
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }
    lastQrScanTime = millis();

    // Read QR
    if (reader.receiveQrCode(&qrCodeData, 100)) {
      if (!qrCodeData.valid) continue;

      String payload = (const char*)qrCodeData.payload;

      // 5-second QR lock
      if (payload == lastScannedPayload &&
          millis() - qrLockTime < qrLockDuration) continue;

      lastScannedPayload = payload;
      qrLockTime = millis();

      Serial.println("Scanned new QRCode");
      Serial.print("Valid payload: ");
      Serial.println(payload);

      int trainIndex = payload.indexOf("Train:");
      int dateIndex  = payload.indexOf("Date:");
      if (trainIndex != -1 && dateIndex != -1) {
        String trainName = payload.substring(trainIndex + 6, dateIndex);
        trainName.trim();
        String dateStr = payload.substring(dateIndex + 5);
        dateStr.trim();
        String currentDate = getCurrentDate();

        if (trainName == "Highland Intercity B" && currentDate == dateStr) {
          Serial.println("Valid ticket - opening gate");

          // Turn on green LED to indicate successful scan
          digitalWrite(GREEN_LED_PIN, HIGH);

          openGateSmoothly();
          gateInCycle = true;

          // LED remains ON until buffer is cleared in gate cycle completion
        } else {
          Serial.println("Invalid ticket - no action taken");
        }
      } else {
        Serial.println("Invalid QR format - no action taken");
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Serial.println();

  setupWiFiAndTime();

  // Servo
  gateServo.attach(SERVO_PIN);
  gateServo.write(CLOSE_ANGLE);
  Serial.println("Servo ready (gate closed)");

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Ultrasonic initialized");

  // LED
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  Serial.println("LED ready");

  // QR
  reader.setup();
  reader.beginOnCore(1);
  Serial.println("QRCode reader initialized");

  // Start task
  xTaskCreate(onQrCodeTask, "onQrCode", 4 * 1024, NULL, 4, NULL);
}

// -------------------- Loop --------------------
void loop() {
  // Empty - all logic handled in task
  delay(200);
}
