///////////////////////////////////////////////////////////////////////
//
//      Custom Gate Actuator Control Logic       
//
//      @benhodes
//      11/9/22
//      MIT License
//      All Rights Reserved
//
///////////////////////////////////////////////////////////////////////

#include <ESP8266WiFi.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager

///////////////////////////////////////////////////////////////////////
// WiFi Settings
WiFiServer server(80);
WiFiManager wifiManager;
unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000;  // Check WiFi every 30 seconds
const unsigned long WIFI_CONNECT_TIMEOUT = 180;   // 3 minute timeout for connections
const int MAX_WIFI_RETRIES = 3;

///////////////////////////////////////////////////////////////////////
// Pin settings
const int IN2 = 2;  // red
const int IN1 = 3;  // orange
const int SSD_WRITE = 4; // SS_Pin
const int ENA = 5;  // black
const int IN4 = 6;  // white
const int IN3 = 8;  // green
const int ENB = 9;  // yellow
const int LIMIT_SWITCH_1_PIN = 10;  // Limit switch for gate 1
const int LIMIT_SWITCH_2_PIN = 11;  // Limit switch for gate 2

// Limit switch and calibration settings
const unsigned long DEBOUNCE_DELAY = 50;    // Debounce time in milliseconds
const int SAFETY_REVERSE_CYCLES = 2;        // Number of cycles to reverse when limit hit
const int SAFE_DISTANCE_CYCLES = 3;         // Target distance from physical limit

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

///////////////////////////////////////////////////////////////////////
// Actuator Position tracking
int A1_POS = 0;
int A2_POS = 0;

///////////////////////////////////////////////////////////////////////
// Calibration values
const int CALIBRATION_SPEED = 150;
const int OPEN_MSEC = 1000;
const int CLOSE_MSEC = 1000;
const int A_SPEED = 160;
const int A1_SPEED = 160;
const int A2_SPEED = 142;
const int A1_ARC_SPEED = 6;
const int A2_ARC_SPEED = 6;
const int A1_MAX_LIMIT = 100;
const int A2_MAX_LIMIT = 100;

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
  pinMode(SSD_WRITE, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLUP);  // Use internal pull-up for normally open switch
  pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLUP);  // Use internal pull-up for normally open switch

  Serial.begin(9600);
  while (!Serial) {};

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  // Initialize WiFiManager
  wifiManager.setConfigPortalTimeout(300); // 5 minutes timeout for configuration portal
  wifiManager.setConnectTimeout(WIFI_CONNECT_TIMEOUT);
  
  // Attempt to connect to saved WiFi or start configuration portal
  if (!wifiManager.autoConnect("GateController-AP")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    ESP.restart(); // Reset and try again
  }

  Serial.println("WiFi connected successfully!");
  printWifiStatus();
  initializeActuators();
  server.begin();
  Serial.println("[SUCCESS] Gate service started");
}

void WiFiConnect() {
  static int retryCount = 0;
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Attempting to reconnect...");
    
    if (retryCount < MAX_WIFI_RETRIES) {
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(); // Attempt to reconnect with saved credentials
      
      unsigned long startAttempt = millis();
      while (WiFi.status() != WL_CONNECTED && 
             millis() - startAttempt < WIFI_CONNECT_TIMEOUT * 1000) {
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
      ESP.restart();
    }
  }
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

void Open_Gates(int Speed) {
  while (A1_POS + A1_ARC_SPEED <= A1_MAX_LIMIT) { A1_Forward(A1_SPEED); }
  while (A2_POS + A2_ARC_SPEED <= A2_MAX_LIMIT) { A2_Forward(A2_SPEED); }
}

void Close_Gates(int Speed) {
  while (A1_POS >= A1_ARC_SPEED) { A1_Backward(A1_SPEED); }
  while (A2_POS >= A2_ARC_SPEED) { A2_Backward(A2_SPEED); }
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
  Serial.print("[Info] Starting gate ");
  Serial.print(gateNum);
  Serial.println(" calibration...");
  
  int cycleCount = 0;
  bool& isCalibrated = (gateNum == 1) ? isCalibrated1 : isCalibrated2;
  isCalibrated = false;
  
  // Reset position
  if (gateNum == 1) {
    while (A1_POS > 0) {
      A1_Backward(CALIBRATION_SPEED);
    }
  } else {
    while (A2_POS > 0) {
      A2_Backward(CALIBRATION_SPEED);
    }
  }
  
  // Move forward until limit switch is triggered
  bool limitReached = false;
  while (!limitReached && cycleCount < 100) { // Safety limit of 100 cycles
    if (gateNum == 1) {
      limitReached = checkLimitSwitch1();
      if (!limitReached) {
        A1_Forward(CALIBRATION_SPEED);
        cycleCount++;
      }
    } else {
      limitReached = checkLimitSwitch2();
      if (!limitReached) {
        A2_Forward(CALIBRATION_SPEED);
        cycleCount++;
      }
    }
  }
  
  if (cycleCount >= 100) {
    Serial.print("[Error] Calibration failed for gate ");
    Serial.print(gateNum);
    Serial.println(" - limit switch not detected");
    return;
  }
  
  if (gateNum == 1) {
    calibratedMaxCycles1 = cycleCount;
  } else {
    calibratedMaxCycles2 = cycleCount;
  }
  isCalibrated = true;
  
  // Move back to safe position
  for (int i = 0; i < SAFETY_REVERSE_CYCLES + SAFE_DISTANCE_CYCLES; i++) {
    if (gateNum == 1) {
      A1_Backward(CALIBRATION_SPEED);
    } else {
      A2_Backward(CALIBRATION_SPEED);
    }
  }
  
  Serial.print("[Success] Gate ");
  Serial.print(gateNum);
  Serial.print(" calibrated. Max cycles: ");
  Serial.println(gateNum == 1 ? calibratedMaxCycles1 : calibratedMaxCycles2);
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
  analogWrite(ENA, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void Motor2_Forward(int Speed) {
  analogWrite(ENB, Speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Motor1_Backward(int Speed) {
  analogWrite(ENA, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void Motor2_Backward(int Speed) {
  analogWrite(ENB, Speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Motor1_Brake() {
  analogWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void Motor2_Brake() {
  analogWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void initializeActuators() {
  Serial.println("Initializing actuators");
  pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLUP);

  for (int i = 0; i <= 17; i++) {
    A1_Backward(0);
    A2_Backward(0);
  }

  A1_POS = 0;
  A2_POS = 0;

  Serial.println("[Success] Gates initialized to close position");
  
  // Perform initial calibration for both gates
  calibrateGate(1);
  calibrateGate(2);
  
  // Only open gates if both calibrations were successful
  if (isCalibrated1 && isCalibrated2) {
    Serial.println("[Action] Opening gates");
    Open_Gates(150);
    Serial.println("[Success] Gates open");
  } else {
    Serial.println("[Error] Skipping initial opening due to failed calibration");
  }
}