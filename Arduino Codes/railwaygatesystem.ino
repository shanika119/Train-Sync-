#include <Servo.h>

#define TRIG_PIN_1 50
#define ECHO_PIN_1 52
#define TRIG_PIN_2 53
#define ECHO_PIN_2 49
#define TRIG_PIN_3 43
#define ECHO_PIN_3 39
#define TRIG_PIN_4 2
#define ECHO_PIN_4 3

#define LED_PIN 8          // Vehicle/train LED
#define KILL_SWITCH_PIN 46 // Kill switch input pin
#define KILL_LED_PIN 9     // Kill switch LED indicator
#define TOGGLE_SWITCH_PIN 48 // Manual toggle switch pin

Servo gateServoLeft;
Servo gateServoRight;
Servo gateServoOtherSide;

const int TRAIN_DETECT_THRESHOLD_CM = 6;
const int VEHICLE_DETECT_THRESHOLD_CM = 5;
const int UNDER_GATE_VEHICLE_THRESHOLD_CM = 2;

unsigned long trainDetectionStart[2] = {0, 0};
unsigned long sensorNoDetectStart[2] = {0, 0};
unsigned long vehicleClearCheckStart = 0;

bool waitingToCloseGate = false;
bool vehicleHoldMode = false; // When true, gates stay open and LED stays on

// Kill switch LED blinking variables
unsigned long killLedPreviousTime = 0;
const unsigned long killLedBlinkInterval = 500; // Blink every 500ms
bool killLedState = false;

enum TrainSide { NONE, SENSOR_1, SENSOR_2 };
TrainSide initialSensor = NONE;
TrainSide confirmSensor = NONE;

bool gatesClosed = false;
bool confirmSensorConfirmed = false;
bool sensorLockedOut[2] = {false, false};
bool vehicleUnderGateDetected = false;

// Toggle switch variables
bool manualToggleState = false; // true if toggle switch activated (LOW means ON)
bool lastToggleSwitchState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 9999;
  return duration * 0.034 / 2;
}

// Original close gates function (resets sensor states) - used for normal train logic
void closeGates(int angle = 90) {
  Serial.print("Closing gates to angle: ");
  Serial.println(angle);

  gateServoLeft.write(angle);
  gateServoRight.write(angle);
  gateServoOtherSide.write(angle);
  delay(500);

  gatesClosed = true;
  waitingToCloseGate = false;
  confirmSensorConfirmed = false;

  if (initialSensor == SENSOR_1) {
    sensorLockedOut[0] = true;
    sensorLockedOut[1] = false;
  } else if (initialSensor == SENSOR_2) {
    sensorLockedOut[1] = true;
    sensorLockedOut[0] = false;
  }
}

// Original open gates function (resets sensor states) - used for normal train logic
void openGates() {
  Serial.println("Opening gates...");

  gateServoLeft.write(0);
  gateServoRight.write(0);
  gateServoOtherSide.write(0);
  delay(500);

  gatesClosed = false;
  waitingToCloseGate = false;
  vehicleHoldMode = false;

  initialSensor = NONE;
  confirmSensor = NONE;
  trainDetectionStart[0] = 0;
  trainDetectionStart[1] = 0;
  sensorNoDetectStart[0] = 0;
  sensorNoDetectStart[1] = 0;
  confirmSensorConfirmed = false;
  sensorLockedOut[0] = false;
  sensorLockedOut[1] = false;

  digitalWrite(LED_PIN, LOW); // Turn LED OFF when gates open normally
}

// New close gates function for toggle OFF - does NOT reset sensor variables
void closeGatesFull(int angle = 90) {
  Serial.print("Closing gates (full) to angle: ");
  Serial.println(angle);

  gateServoLeft.write(angle);
  gateServoRight.write(angle);
  gateServoOtherSide.write(angle);
  delay(500);

  gatesClosed = true;
  waitingToCloseGate = false;
  // Do NOT reset sensor variables here!
}

// New open gates function for toggle ON - does NOT reset sensor variables
void openGatesFull() {
  Serial.println("Opening gates (full)...");

  gateServoLeft.write(0);
  gateServoRight.write(0);
  gateServoOtherSide.write(0);
  delay(500);

  gatesClosed = false;
  waitingToCloseGate = false;
  vehicleHoldMode = false;

  // Do NOT reset sensor variables here!

  digitalWrite(LED_PIN, LOW); // LED off only if toggle OFF
}

void setup() {
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
  pinMode(TRIG_PIN_4, OUTPUT);
  pinMode(ECHO_PIN_4, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
  pinMode(KILL_LED_PIN, OUTPUT);
  digitalWrite(KILL_LED_PIN, LOW);

  pinMode(TOGGLE_SWITCH_PIN, INPUT_PULLUP);

  gateServoLeft.attach(30);
  gateServoRight.attach(32);
  gateServoOtherSide.attach(44);

  gateServoLeft.write(0);
  gateServoRight.write(0);
  gateServoOtherSide.write(0);

  Serial.begin(9600);
}

void loop() {
  unsigned long now = millis();

  // Kill switch logic with blinking LED
  bool killSwitchReading = digitalRead(KILL_SWITCH_PIN);
  if (killSwitchReading == LOW) {
    // Kill switch ON: disable automation, open gates fully, Kill LED BLINKING
    openGatesFull();    // Fully open gates
    
    // Handle blinking LED
    if (now - killLedPreviousTime >= killLedBlinkInterval) {
      killLedPreviousTime = now;
      killLedState = !killLedState;
      digitalWrite(KILL_LED_PIN, killLedState ? HIGH : LOW);
    }
    
    delay(100);
    return;
  } else {
    digitalWrite(KILL_LED_PIN, LOW);
    killLedState = false; // Reset blink state when kill switch is off
  }

  // Read manual toggle switch
  bool toggleSwitchReading = digitalRead(TOGGLE_SWITCH_PIN);

  // Debounce toggle switch input
  if (toggleSwitchReading != lastToggleSwitchState) {
    lastDebounceTime = now;
  }

  if ((now - lastDebounceTime) > debounceDelay) {
    // Update manualToggleState based on switch position: LOW = ON, HIGH = OFF
    if (manualToggleState != (toggleSwitchReading == LOW)) {
      manualToggleState = (toggleSwitchReading == LOW);

      if (manualToggleState) {
        // Switch ON: open gates fully, LED ON
        openGatesFull();
        digitalWrite(LED_PIN, HIGH);
        Serial.println("Toggle switch: Gates FULLY OPEN, LED ON");
      } else {
        // Switch OFF: close gates fully, LED OFF
        closeGatesFull(90);
        digitalWrite(LED_PIN, LOW);
        Serial.println("Toggle switch: Gates FULLY CLOSED, LED OFF");
      }
    }
  }

  lastToggleSwitchState = toggleSwitchReading;

  // If manual toggle mode active, skip normal sensor logic
  if (manualToggleState) {
    delay(100);
    return;
  }

  // Normal sensor logic below

  long dist1 = readUltrasonicDistance(TRIG_PIN_1, ECHO_PIN_1);
  long dist2 = readUltrasonicDistance(TRIG_PIN_2, ECHO_PIN_2);
  long vehicleDist = readUltrasonicDistance(TRIG_PIN_3, ECHO_PIN_3);
  long underGateDist = readUltrasonicDistance(TRIG_PIN_4, ECHO_PIN_4);

  Serial.print("Sensor 1: ");
  Serial.print(dist1);
  Serial.print(" cm, Sensor 2: ");
  Serial.print(dist2);
  Serial.print(" cm, Vehicle Sensor: ");
  Serial.print(vehicleDist);
  Serial.print(" cm, Under Gate Sensor: ");
  Serial.println(underGateDist);

  // Stabilize under gate detection only when gates are closed
  static bool prevUnderGateDetected = false;
  static unsigned long underGateStableStart = 0;
  const unsigned long underGateStableTime = 500;

  if (gatesClosed) {
    bool currentUnderGateDetected = (underGateDist <= UNDER_GATE_VEHICLE_THRESHOLD_CM);
    if (currentUnderGateDetected != prevUnderGateDetected) {
      if (underGateStableStart == 0) {
        underGateStableStart = now;
      } else if (now - underGateStableStart >= underGateStableTime) {
        vehicleUnderGateDetected = currentUnderGateDetected;
        prevUnderGateDetected = currentUnderGateDetected;
        underGateStableStart = 0;
      }
    } else {
      underGateStableStart = 0;
    }
  } else {
    vehicleUnderGateDetected = underGateDist <= UNDER_GATE_VEHICLE_THRESHOLD_CM;
  }

  // TRAIN DETECTION WHEN GATES OPEN
  if (!gatesClosed) {
    if (!sensorLockedOut[0]) {
      if (dist1 <= TRAIN_DETECT_THRESHOLD_CM) {
        if (trainDetectionStart[0] == 0)
          trainDetectionStart[0] = now;
        if (now - trainDetectionStart[0] >= 3000) {
          if (initialSensor != SENSOR_1) {
            Serial.println("Train detected on Sensor 1 - Preparing to close gates");
            initialSensor = SENSOR_1;
            confirmSensor = SENSOR_2;
            waitingToCloseGate = true;
            vehicleClearCheckStart = now;
          }
        }
      } else {
        trainDetectionStart[0] = 0;
      }
    }

    if (initialSensor == NONE && !sensorLockedOut[1]) {
      if (dist2 <= TRAIN_DETECT_THRESHOLD_CM) {
        if (trainDetectionStart[1] == 0)
          trainDetectionStart[1] = now;
        if (now - trainDetectionStart[1] >= 3000) {
          Serial.println("Train detected on Sensor 2 - Preparing to close gates");
          initialSensor = SENSOR_2;
          confirmSensor = SENSOR_1;
          waitingToCloseGate = true;
          vehicleClearCheckStart = now;
        }
      } else {
        trainDetectionStart[1] = 0;
      }
    }
  }

  // WAIT FOR VEHICLE CLEARANCE BEFORE CLOSING GATES FOR TRAIN
  if (waitingToCloseGate && !gatesClosed && !vehicleHoldMode) {
    if (vehicleDist > VEHICLE_DETECT_THRESHOLD_CM) {
      if (now - vehicleClearCheckStart >= 3000) {
        Serial.println("No vehicle blocking, closing gates now.");
        closeGates();  // Default 90 degrees for train, resets sensors
        waitingToCloseGate = false;
      }
    } else {
      vehicleClearCheckStart = now;
      Serial.println("Vehicle blocking gate, waiting to close.");
    }
  }

  // CONFIRM TRAIN PASSING AND OPEN GATES
  if (gatesClosed && initialSensor != NONE) {
    int confirmIndex = (confirmSensor == SENSOR_1) ? 0 : 1;
    long confirmDist = (confirmSensor == SENSOR_1) ? dist1 : dist2;

    if (!confirmSensorConfirmed) {
      if (confirmDist <= TRAIN_DETECT_THRESHOLD_CM) {
        if (trainDetectionStart[confirmIndex] == 0)
          trainDetectionStart[confirmIndex] = now;
        if (now - trainDetectionStart[confirmIndex] >= 5000) {
          Serial.println("Confirm sensor triggered, train confirmed passing.");
          confirmSensorConfirmed = true;
          sensorNoDetectStart[confirmIndex] = 0;
        }
      } else {
        trainDetectionStart[confirmIndex] = 0;
      }
    } else {
      if (confirmDist > TRAIN_DETECT_THRESHOLD_CM) {
        if (sensorNoDetectStart[confirmIndex] == 0)
          sensorNoDetectStart[confirmIndex] = now;
        if (now - sensorNoDetectStart[confirmIndex] >= 3000) {
          Serial.println("Train passed, opening gates.");
          openGates();  // Resets sensor variables after train leaves
        }
      } else {
        sensorNoDetectStart[confirmIndex] = 0;
      }
    }
  }

  // VEHICLE DETECTION AND LED LOGIC — ACTIVE ONLY WHEN TRAIN DETECTED OR GATES CLOSED
  static bool ledState = false;
  static unsigned long ledOnStart = 0;
  static bool gateClosedForVehicle = false;

  if (waitingToCloseGate || gatesClosed) {
    bool vehicleDetected = (vehicleDist <= VEHICLE_DETECT_THRESHOLD_CM) || (underGateDist <= UNDER_GATE_VEHICLE_THRESHOLD_CM);

    if (vehicleDetected) {
      if (ledOnStart == 0) {
        ledOnStart = now;
      } else {
        if (!ledState && (now - ledOnStart >= 3000)) {
          ledState = true;
          digitalWrite(LED_PIN, HIGH);  // LED ON

          if (!gateClosedForVehicle) {
            closeGates(20);  // Partially close gates for vehicle
            gateClosedForVehicle = true;
            vehicleHoldMode = true;  // Hold gates closed and LED ON
            Serial.println("Vehicle detected (sensor 3 or 4), gates partially closed and LED ON.");
          }
        }
      }
    } else {
      ledOnStart = 0;
      // Keep LED ON and gates as is until gates open explicitly
    }
  } else {
    // No train detected, so ignore vehicle sensors
    ledOnStart = 0;
    if (ledState) {
      ledState = false;
      digitalWrite(LED_PIN, LOW);  // Make sure LED is OFF when no train detected
    }
  }

  delay(100);
}