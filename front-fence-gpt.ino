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

///////////////////////////////////////////////////////////////////////
// WiFi Settings
const char* ssid = "Fort Awesome";   // your network SSID (name)
const char* pass = "3Py605fa#0V0";   // your network password
WiFiServer server(80);
int status = WL_IDLE_STATUS;    // the Wifi radio's status

///////////////////////////////////////////////////////////////////////
// Pin settings
const int IN2 = 2;  // red
const int IN1 = 3;  // orange
const int SSD_WRITE = 4; // SS_Pin
const int ENA = 5;  // black
const int IN4 = 6;  // white
const int IN3 = 8;  // green
const int ENB = 9;  // yellow

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

  Serial.begin(9600);
  while (!Serial) {};

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  WiFiConnect();
  printWifiStatus();
  initializeActuators();
  server.begin();
  Serial.println("[SUCCESS] Gate service started");
}

void WiFiConnect() {
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(2000);
  }
}

void loop() {
  if (status != WL_CONNECTED) { WiFiConnect(); }
  WiFiClient client = server.available();

  delay(50);

  if (client) {
    currentAction = NONE;
    HTTP_req = "";

    Serial.println("New request");
    bool currentLineIsBlank = true;

    delay(50);

    while (client.connected()) {
      if (!client) { break; }
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

void A1_Forward(int Speed) {
  if (A1_POS + A1_ARC_SPEED <= A1_MAX_LIMIT) {
    A1_POS += A1_ARC_SPEED;
    Motor1_Forward(Speed);
    delay(OPEN_MSEC);
    Motor1_Brake();
  }
}

void A2_Forward(int Speed) {
  if (A2_POS + A2_ARC_SPEED <= A2_MAX_LIMIT) {
    A2_POS += A2_ARC_SPEED;
    Motor2_Forward(Speed);
    delay(OPEN_MSEC);
    Motor2_Brake();
  }
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

  for (int i = 0; i <= 17; i++) {
    A1_Backward(0);
    A2_Backward(0);
  }

  A1_POS = 0;
  A2_POS = 0;

  Serial.println("[Success] Gate initialized to close position");
  Serial.println("[Action] Opening gates");
  Open_Gates(150);
  Serial.println("[Success] Gates open");
}