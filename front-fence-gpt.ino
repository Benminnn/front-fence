///////////////////////////////////////////////////////////////////////
//
//      Custom Gate Actuator Control Logic       
//
//      @benhodes
//      11/9/22
//      MIT License
//      All Rights Reserved
//
//      Updated: 3/9/25
//
///////////////////////////////////////////////////////////////////////

#include <WiFiNINA.h>

// Check for config file
#if __has_include("config.h")
#include "config.h"
#else
#error "Configuration file not found. Please copy config.h.template to config.h and update with your credentials"
#endif

///////////////////////////////////////////////////////////////////////
// WiFi Settings
WiFiServer server(80);
unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000;  // Check WiFi every 30 seconds
const unsigned long WIFI_CONNECT_TIMEOUT = 180000; // 3 minute timeout for connections
const int MAX_WIFI_RETRIES = 3;

///////////////////////////////////////////////////////////////////////
// Pin settings
const int IN2 = 2;  // red
const int IN1 = 3;  // orange

const int SSD_WRITE = 4; // SS_Pin

const int ENA = 5;  // black
const int IN4 = 6;  // white
// 7 wifi reserved
const int IN3 = 8;  // green
const int ENB = 9;  // yellow
// 10 wifi reserved
// 11 wifi reserved
// 12 wifi reserved

// analog pins A0 + 14
const int LIMIT_SWITCH_1_PIN = 14;  // Limit switch for gate 1 (A0)
const int LIMIT_SWITCH_2_PIN = 15;  // Limit switch for gate 2 (A1)
const int KEYPAD_PIN = 16;  // Keypad input (A2)

// Limit switch and calibration settings
const unsigned long DEBOUNCE_DELAY = 50;    // Debounce time in milliseconds
const int SAFETY_REVERSE_CYCLES = 2;        // Number of cycles to reverse when limit hit
const int SAFE_DISTANCE_CYCLES = 3;         // Target distance from physical limit

// Keypad settings
const unsigned long MIN_KEYPAD_DURATION = 1000;  // Minimum duration (ms) to consider valid keypad input
unsigned long keypadActivationTime = 0;          // Time when keypad signal was first detected
bool keypadActive = false;                       // Current keypad state
int lastKeypadState = LOW;                      // Previous keypad reading
unsigned long lastKeypadDebounceTime = 0;        // Last time the keypad input changed
bool gateAccessAuthorized = false;               // Flag for authorized gate access

// Gate state tracking
bool isGateOpen = false;                        // Track if gate is fully open

// Gate 1 limit switch state
unsigned long lastDebounceTime1 = 0;        // Last time the limit switch 1 was toggled
int lastLimitSwitch1State = HIGH;          // Previous reading from limit switch 1
int limitSwitch1State = HIGH;              // Current debounced state
int calibratedMaxCycles1 = 0;              // Calibrated cycles for gate 1
bool isCalibrated1 = false;                // Whether gate 1 has been calibrated

// Gate 2 limit switch state
unsigned long lastDebounceTime2 = 0;        // Last time the limit switch 2 was toggled
int lastLimitSwitch2State = HIGH;          // Previous reading from limit switch 2
int limitSwitch2State = HIGH;              // Current debounced state
int calibratedMaxCycles2 = 0;              // Calibrated cycles for gate 2
bool isCalibrated2 = false;                // Whether gate 2 has been calibrated

// Global state for position initialization
bool positionInitialized = false;

///////////////////////////////////////////////////////////////////////
// Actuator Position tracking
int A1_POS = -1;  // -1 indicates unknown position
int A2_POS = -1;

///////////////////////////////////////////////////////////////////////
// Calibration values
const int CALIBRATION_SPEED = 150;
const int OPEN_MSEC = 1000;
const int CLOSE_MSEC = 1000;
const int A_SPEED = 160;  // Base speed for both motors
const int A1_SPEED = A_SPEED;  // Use base speed for consistency
const int A2_SPEED = A_SPEED;  // Use base speed for consistency
const int A1_ARC_SPEED = 6;
const int A2_ARC_SPEED = 6;
const int A1_MAX_LIMIT = 100;  // Maximum position limit for gate 1
const int A2_MAX_LIMIT = 100;  // Maximum position limit for gate 2

///////////////////////////////////////////////////////////////////////
// Action Enum
enum Action {
  NONE,
  VIEW,
  OPEN,
  STEP_OPEN,
  STEP_CLOSE,
  A1_OPEN,
  A1_CLOSE,
  A2_OPEN,
  A2_CLOSE,
  CLOSE,
  HALT
};

Action currentAction = NONE;
String HTTP_req = "";

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {};

  // Set up hardware pins
  pinMode(SSD_WRITE, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLUP);  // Use internal pull-up for normally open switch
  pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLUP);  // Use internal pull-up for normally open switch
  pinMode(KEYPAD_PIN, INPUT);  // Keypad input

  // Check WiFi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("Communication with WiFi module failed!"));
    while (true);
  }

  // Check firmware version
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println(F("Please upgrade the firmware"));
  }

  // Initialize WiFi connection with retry logic
  WiFiConnect();
  
  // Initialize actuators and calibrate
  initializeActuators();
  
  // Start web server
  server.begin();
  Serial.println(F("[SUCCESS] Gate service started"));
}

void loop() {
  // Periodic WiFi connection check
  unsigned long currentMillis = millis();
  if (currentMillis - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFiConnect();
    }
    lastWifiCheck = currentMillis;
  }

  if (!positionInitialized) {
    initializePosition();
    return;
  }

  // Check keypad access
  checkKeypadAccess();
  
  WiFiClient client = server.available();
  
  if (client) {
    if (!client.connected()) {
      Serial.println("Client disconnected");
      client.stop();
      return;
    }
    
    currentAction = NONE;
    HTTP_req = "";

    Serial.println("New request");
    bool currentLineIsBlank = true;
    unsigned long timeout = millis();

    while (client.connected()) {
      // Add timeout for stuck connections
      if (millis() - timeout > 5000) { // 5 second timeout
        Serial.println("Client connection timeout");
        break;
      }
      
      if (client.available()) {
        char c = client.read();
        HTTP_req += c;

        if (c == '\n' && currentLineIsBlank) {
          determineAction();
          sendHttpResponse(client);
          break;
        }

        if (c == '\n') {
          currentLineIsBlank = true;
        } else if (c != '\r') {
          currentLineIsBlank = false;
        }
        
        timeout = millis(); // Reset timeout counter on received data
      }
    }

    delay(10);
    client.stop();
    executeAction();
  }

  delay(10);
}

void determineAction() {
  if (HTTP_req.indexOf("GET / HTTP/1.1") >= 0) {
    currentAction = VIEW;
  } else if (HTTP_req.indexOf("GET /action/open/ HTTP/1.1") >= 0) {
    currentAction = OPEN;
  } else if (HTTP_req.indexOf("GET /action/step_open/ HTTP/1.1") >= 0) {
    currentAction = STEP_OPEN;
  } else if (HTTP_req.indexOf("GET /action/step_close/ HTTP/1.1") >= 0) {
    currentAction = STEP_CLOSE;
  } else if (HTTP_req.indexOf("GET /action/a1_open/ HTTP/1.1") >= 0) {
    currentAction = A1_OPEN;
  } else if (HTTP_req.indexOf("GET /action/a1_close/ HTTP/1.1") >= 0) {
    currentAction = A1_CLOSE;
  } else if (HTTP_req.indexOf("GET /action/a2_open/ HTTP/1.1") >= 0) {
    currentAction = A2_OPEN;
  } else if (HTTP_req.indexOf("GET /action/a2_close/ HTTP/1.1") >= 0) {
    currentAction = A2_CLOSE;
  } else if (HTTP_req.indexOf("GET /action/close/ HTTP/1.1") >= 0) {
    currentAction = CLOSE;
  } else if (HTTP_req.indexOf("GET /action/halt/ HTTP/1.1") >= 0) {
    currentAction = HALT;
  }
}

void sendHttpResponse(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
}

void executeAction() {
  switch (currentAction) {
    case OPEN:
      Open_Gates(A_SPEED);
      break;
    case CLOSE:
      Close_Gates(A_SPEED);
      break;
    case STEP_OPEN:
      A1_Forward(A1_SPEED);
      A2_Forward(A2_SPEED);
      break;
    case STEP_CLOSE:
      A1_Backward(A1_SPEED);
      A2_Backward(A2_SPEED);
      break;
    case A1_OPEN:
      A1_Forward(A1_SPEED);
      break;
    case A1_CLOSE:
      A1_Backward(A1_SPEED);
      break;
    case A2_OPEN:
      A2_Forward(A2_SPEED);
      break;
    case A2_CLOSE:
      A2_Backward(A2_SPEED);
      break;
    default:
      break;
  }

  Serial.print("[ACTION] Client request served: ");
  Serial.println(currentAction);
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void initializeActuators() {
  Serial.println(F("[Init] Starting safe position initialization..."));

  // Set initial state
  isGateOpen = false;
  gateAccessAuthorized = false;
  A1_POS = -1;  // Start at unknown position
  A2_POS = -1;
  isCalibrated1 = false;
  isCalibrated2 = false;
  currentAction = NONE;

  // Initialize motor control pins to safe state
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Find zero position
  Serial.println(F("[Init] Finding zero position - moving gates inward..."));
  
  // Move gates inward until limit switches trigger
  while (!checkLimitSwitch1() || !checkLimitSwitch2()) {
    if (!checkLimitSwitch1()) {
      Motor1_Backward(CALIBRATION_SPEED);
    } else {
      Stop_A1();
    }
    
    if (!checkLimitSwitch2()) {
      Motor2_Backward(CALIBRATION_SPEED);
    } else {
      Stop_A2();
    }
  }

  // Stop both motors
  Stop_A1();
  Stop_A2();

  // Set initial position
  A1_POS = 0;
  A2_POS = 0;
  positionInitialized = true;

  Serial.println(F("[Init] Zero position found - gates initialized"));

  // Calibrate both gates
  Serial.println(F("[Info] Starting gate 1 calibration..."));
  calibrateGate(1);
  
  Serial.println(F("[Info] Starting gate 2 calibration..."));
  calibrateGate(2);
}

void WiFiConnect() {
  static int retryCount = 0;
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Attempting to reconnect...");
    
    if (retryCount < MAX_WIFI_RETRIES) {
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      
      unsigned long startAttempt = millis();
      while (WiFi.status() != WL_CONNECTED && 
             millis() - startAttempt < WIFI_CONNECT_TIMEOUT) {
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() != WL_CONNECTED) {
        retryCount++;
        Serial.println("\nWiFi reconnection attempt failed");
      } else {
        retryCount = 0; // Reset counter on successful connection
        Serial.println("\nWiFi reconnected successfully");
        printWifiStatus();
      }
    } else {
      Serial.println("Maximum WiFi retry attempts reached. Restarting device...");
      delay(1000);
      // Reset the board
      void(*resetFunc)(void) = 0;
      resetFunc();
    }
  }
}

void initializePosition() {
  if (positionInitialized) {
    return;
  }
  
  Serial.println("[Init] Finding zero position - moving gates inward...");
  
  // Safely move both gates inward until they hit their physical limits
  // This is safe as the physical gate structure prevents damage when closing
  while (!checkLimitSwitch1() || !checkLimitSwitch2()) {
    if (!checkLimitSwitch1()) {
      A1_Backward(A1_SPEED);
    } else {
      Stop_A1();
    }
    
    if (!checkLimitSwitch2()) {
      A2_Backward(A2_SPEED);
    } else {
      Stop_A2();
    }
    
    delay(10);  // Small delay to prevent overwhelming the system
  }
  
  // Both gates have hit their limit switches - we know we're at position 0
  Stop_A1();
  Stop_A2();
  A1_POS = 0;
  A2_POS = 0;
  positionInitialized = true;
  isGateOpen = false;
  
  Serial.println("[Init] Zero position found - gates initialized");
  
  // Perform initial calibration for both gates
  calibrateGate(1);
  calibrateGate(2);
  
  // Only open gates if both calibrations were successful
  if (isCalibrated1 && isCalibrated2) {
    Serial.println("[Action] Opening gates");
    gateAccessAuthorized = true;  // Allow initial opening
    Open_Gates(150);
    Serial.println("[Success] Gates open");
  } else {
    Serial.println("[Error] Skipping initial opening due to failed calibration");
  }
}

void Open_Gates(int Speed) {
  if (!isCalibrated1 || !isCalibrated2) {
    Serial.println(F("[Error] Gates not calibrated - cannot open"));
    return;
  }

  if (!gateAccessAuthorized) {
    Serial.println(F("[Error] Access not authorized"));
    return;
  }

  Serial.println(F("[Action] Opening gates..."));
  
  // Check limit switches before moving
  if (checkLimitSwitch1() || checkLimitSwitch2()) {
    Serial.println(F("[Warning] Limit switch triggered - cannot open"));
    return;
  }

  // Set motor speeds
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);

  // Set direction for opening
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Update state
  isGateOpen = true;
  gateAccessAuthorized = false;  // Reset authorization after use

  // Log final positions
  Serial.print(F("[Info] Final positions - Gate 1: "));
  Serial.print(A1_POS);
  Serial.print(F(", Gate 2: "));
  Serial.println(A2_POS);

  Serial.println(F("[Success] Gates open"));
}

void Close_Gates(int Speed) {
  if (!isGateOpen) {
    Serial.println(F("[Error] Gates already closed"));
    return;
  }

  Serial.println(F("[Action] Closing gates..."));

  // Set motor speeds
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);

  // Set direction for closing
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Update state
  isGateOpen = false;
  A1_POS = 0;  // Reset positions when fully closed
  A2_POS = 0;

  Serial.println(F("[Success] Gates fully closed"));
}

// Position clamping function
void clampPositions() {
  // Clamp A1_POS
  if (A1_POS > A1_MAX_LIMIT) {
    A1_POS = A1_MAX_LIMIT;
  } else if (A1_POS < 0) {
    A1_POS = 0;
  }

  // Clamp A2_POS
  if (A2_POS > A2_MAX_LIMIT) {
    A2_POS = A2_MAX_LIMIT;
  } else if (A2_POS < 0) {
    A2_POS = 0;
  }
}

void checkKeypadAccess() {
  int reading = digitalRead(KEYPAD_PIN);
  unsigned long currentMillis = millis();

  // Check if reading has changed
  if (reading != lastKeypadState) {
    lastKeypadDebounceTime = currentMillis;
  }

  // If enough time has passed, update state
  if ((currentMillis - lastKeypadDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != keypadActive) {
      keypadActive = reading;

      if (keypadActive == HIGH) {
        // Start timing keypad press
        keypadActivationTime = currentMillis;
      } else {
        // Check if press was long enough
        if ((currentMillis - keypadActivationTime) >= MIN_KEYPAD_DURATION) {
          gateAccessAuthorized = true;
          currentAction = isGateOpen ? CLOSE : OPEN;
        }
      }
    }
  }

  lastKeypadState = reading;
}

bool debounceDigitalRead(int pin, int& lastState, unsigned long& lastDebounceTime) {
  int reading = digitalRead(pin);
  bool stateChanged = false;

  // Reset debounce timer if input changed
  if (reading != lastState) {
    lastDebounceTime = millis();
  }

  // Check if enough time has passed since last change
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // Only update if reading is different from current state
    if (reading != limitSwitch1State && pin == LIMIT_SWITCH_1_PIN) {
      limitSwitch1State = reading;
      stateChanged = true;
    } else if (reading != limitSwitch2State && pin == LIMIT_SWITCH_2_PIN) {
      limitSwitch2State = reading;
      stateChanged = true;
    }
  }

  lastState = reading;
  return stateChanged;
}

bool checkLimitSwitch1() {
  if (debounceDigitalRead(LIMIT_SWITCH_1_PIN, lastLimitSwitch1State, lastDebounceTime1)) {
    return (limitSwitch1State == LOW);
  }
  return (limitSwitch1State == LOW);
}

bool checkLimitSwitch2() {
  if (debounceDigitalRead(LIMIT_SWITCH_2_PIN, lastLimitSwitch2State, lastDebounceTime2)) {
    return (limitSwitch2State == LOW);
  }
  return (limitSwitch2State == LOW);
}

void handleLimitSwitchTrigger(int gateNum) {
  // Stop immediately
  if (gateNum == 1) {
    Motor1_Brake();
  } else {
    Motor2_Brake();
  }
  Serial.print("[Warning] Limit switch ");
  Serial.print(gateNum);
  Serial.println(" triggered - reversing");
  
  // Reverse for safety
  for (int i = 0; i < SAFETY_REVERSE_CYCLES; i++) {
    if (gateNum == 1) {
      A1_Backward(A1_SPEED);
    } else {
      A2_Backward(A2_SPEED);
    }
  }
}

void calibrateGate(int gateNum) {
  int cycles = 0;
  bool limitFound = false;
  
  // Reset calibration state
  if (gateNum == 1) {
    isCalibrated1 = false;
    calibratedMaxCycles1 = 0;
  } else {
    isCalibrated2 = false;
    calibratedMaxCycles2 = 0;
  }

  // Move gate outward until limit switch triggers
  while (!limitFound && cycles < 200) {  // Safety limit of 200 cycles
    if (gateNum == 1) {
      if (!checkLimitSwitch1()) {
        Motor1_Forward(CALIBRATION_SPEED);
        cycles++;
      } else {
        Stop_A1();
        limitFound = true;
        calibratedMaxCycles1 = cycles;
        isCalibrated1 = true;
        A1_POS = A1_MAX_LIMIT;  // At maximum position
      }
    } else {
      if (!checkLimitSwitch2()) {
        Motor2_Forward(CALIBRATION_SPEED);
        cycles++;
      } else {
        Stop_A2();
        limitFound = true;
        calibratedMaxCycles2 = cycles;
        isCalibrated2 = true;
        A2_POS = A2_MAX_LIMIT;  // At maximum position
      }
    }
    delay(50);  // Small delay to prevent overwhelming the system
  }

  // Stop motors and report results
  if (gateNum == 1) {
    Stop_A1();
    if (limitFound) {
      Serial.print(F("[Success] Gate 1 calibrated. Max cycles: "));
      Serial.println(calibratedMaxCycles1);
    } else {
      Serial.println(F("[Error] Gate 1 calibration failed - limit not found"));
      isCalibrated1 = false;
    }
  } else {
    Stop_A2();
    if (limitFound) {
      Serial.print(F("[Success] Gate 2 calibrated. Max cycles: "));
      Serial.println(calibratedMaxCycles2);
    } else {
      Serial.println(F("[Error] Gate 2 calibration failed - limit not found"));
      isCalibrated2 = false;
    }
  }

  // Move gate back to closed position
  if (limitFound) {
    if (gateNum == 1) {
      while (!checkLimitSwitch1()) {
        Motor1_Backward(CALIBRATION_SPEED);
      }
      Stop_A1();
      A1_POS = 0;  // Back at closed position
    } else {
      while (!checkLimitSwitch2()) {
        Motor2_Backward(CALIBRATION_SPEED);
      }
      Stop_A2();
      A2_POS = 0;  // Back at closed position
    }
  }
}

void A1_Forward(int Speed) {
  // Check limit switch first
  if (checkLimitSwitch1()) {
    handleLimitSwitchTrigger(1);
    return;
  }
  
  // Check if we're approaching the calibrated limit
  if (isCalibrated1 && A1_POS >= (calibratedMaxCycles1 - SAFE_DISTANCE_CYCLES)) {
    Serial.println("[Warning] A1 approaching calibrated limit");
    return;
  }
  
  // Validate position bounds before moving
  if (A1_POS >= A1_MAX_LIMIT) {
    Serial.println("[Warning] A1 at maximum limit, cannot move forward");
    return;
  }
  
  int new_pos = A1_POS + A1_ARC_SPEED;
  if (new_pos > A1_MAX_LIMIT) {
    new_pos = A1_MAX_LIMIT;
  }
  
  A1_POS = new_pos;
  Motor1_Forward(Speed);
  delay(OPEN_MSEC);
  Motor1_Brake();
  Serial.print("[Info] A1 position: ");
  Serial.println(A1_POS);
}

void A2_Forward(int Speed) {
  // Check limit switch first
  if (checkLimitSwitch2()) {
    handleLimitSwitchTrigger(2);
    return;
  }
  
  // Check if we're approaching the calibrated limit
  if (isCalibrated2 && A2_POS >= (calibratedMaxCycles2 - SAFE_DISTANCE_CYCLES)) {
    Serial.println("[Warning] A2 approaching calibrated limit");
    return;
  }
  
  // Validate position bounds before moving
  if (A2_POS >= A2_MAX_LIMIT) {
    Serial.println("[Warning] A2 at maximum limit, cannot move forward");
    return;
  }
  
  int new_pos = A2_POS + A2_ARC_SPEED;
  if (new_pos > A2_MAX_LIMIT) {
    new_pos = A2_MAX_LIMIT;
  }
  
  A2_POS = new_pos;
  Motor2_Forward(Speed);
  delay(OPEN_MSEC);
  Motor2_Brake();
  Serial.print("[Info] A2 position: ");
  Serial.println(A2_POS);
}

void A1_Backward(int Speed) {
  if (!Speed || A1_POS >= A1_ARC_SPEED) {
    A1_POS -= A1_ARC_SPEED;
    int local_speed = Speed ? Speed : CALIBRATION_SPEED;
    Motor1_Backward(local_speed);
    delay(CLOSE_MSEC);
    Motor1_Brake();
  }
}

void A2_Backward(int Speed) {
  if (!Speed || A2_POS >= A2_ARC_SPEED) {
    A2_POS -= A2_ARC_SPEED;
    int local_speed = Speed ? Speed : CALIBRATION_SPEED;
    Motor2_Backward(local_speed);
    delay(CLOSE_MSEC);
    Motor2_Brake();
  }
}

void Motor1_Forward(int Speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, Speed);
  A1_POS++;
  clampPositions();
}

void Motor2_Forward(int Speed) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, Speed);
  A2_POS++;
  clampPositions();
}

void Motor1_Backward(int Speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, Speed);
  A1_POS--;
  clampPositions();
}

void Motor2_Backward(int Speed) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, Speed);
  A2_POS--;
  clampPositions();
}

void Motor1_Brake() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void Motor2_Brake() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void Stop_A1() {
  Motor1_Brake();
}

void Stop_A2() {
  Motor2_Brake();
}