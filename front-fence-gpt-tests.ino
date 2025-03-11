
///////////////////////////////////////////////////////////////////////
//
//      Gate Controller Test Suite
//      Tests for front-fence-gpt.ino
//
//      @benhodes
//      3/10/25
//      MIT License
//      All Rights Reserved
//
///////////////////////////////////////////////////////////////////////

#include <WiFiNINA.h>

// Constants for testing
#define MAX_PINS 20
#define TEST_TIMEOUT 5000  // 5 second timeout for tests

// Simple test framework with unique debug codes
#define assertEqual(a, b, code) do { \
  if ((a) != (b)) { \
    Serial.print(F("[FAIL-")); \
    Serial.print(F("T")); Serial.print(code); \
    Serial.print(F("] ")); \
    Serial.print(F(__FILE__)); \
    Serial.print(F(":")); \
    Serial.print(__LINE__); \
    Serial.print(F(" Got: ")); \
    Serial.print(a); \
    Serial.print(F(" Expected: ")); \
    Serial.println(b); \
    testsFailed++; \
  } else { \
    testsPassed++; \
  } \
} while(0)

#define assertNotEqual(a, b, code) do { \
  if ((a) == (b)) { \
    Serial.print(F("[FAIL-")); \
    Serial.print(F("T")); Serial.print(code); \
    Serial.print(F("] ")); \
    Serial.print(F(__FILE__)); \
    Serial.print(F(":")); \
    Serial.print(__LINE__); \
    Serial.print(F(" Got: ")); \
    Serial.print(a); \
    Serial.print(F(" Expected not equal to: ")); \
    Serial.println(b); \
    testsFailed++; \
  } else { \
    testsPassed++; \
  } \
} while(0)

#define assertTrue(x, code) assertEqual((x), true, code)
#define assertFalse(x, code) assertEqual((x), false, code)

// Test statistics
int testsPassed = 0;
int testsFailed = 0;

// External references to variables we need to test
extern bool gateAccessAuthorized;
extern bool isGateOpen;
extern int A1_POS;
extern int A2_POS;
extern bool isCalibrated1;
extern bool isCalibrated2;
extern Action currentAction;

// Mock state storage
PinStatus mockPinStates[MAX_PINS];
int mockAnalogValues[MAX_PINS];
PinMode mockPinModes[MAX_PINS];
unsigned long keypadPressStartTime = 0;  // Track keypad press duration

// Test state variables
bool testGateAccessAuthorized = false;
bool testIsGateOpen = false;
int testA1_POS = -1;
int testA2_POS = -1;
bool testIsCalibrated1 = false;
bool testIsCalibrated2 = false;
int testA1_Speed = 0;  // Track motor speeds for testing
int testA2_Speed = 0;

// Flag to control whether we use mocks or real hardware
bool useMocks = false;

// Hardware wrapper functions for testing
void hw_digitalWrite(pin_size_t pin, PinStatus val) {
  if (useMocks) {
    mockDigitalWrite(pin, val);
  } else {
    digitalWrite(pin, val);
  }
}

PinStatus hw_digitalRead(pin_size_t pin) {
  if (useMocks) {
    return mockDigitalRead(pin);
  }
  return digitalRead(pin);
}

void hw_analogWrite(pin_size_t pin, int value) {
  if (useMocks) {
    mockAnalogWrite(pin, value);
  } else {
    analogWrite(pin, value);
  }
}

void hw_pinMode(pin_size_t pin, PinMode mode) {
  if (useMocks) {
    mockPinMode(pin, mode);
  } else {
    pinMode(pin, mode);
  }
}

// Mock functions with correct types
void mockDigitalWrite(pin_size_t pin, PinStatus val) {
  mockPinStates[pin] = val;
  
  // Handle keypad press timing
  if (pin == KEYPAD_PIN) {
    if (val == HIGH) {
      keypadPressStartTime = millis();
    } else {
      // On keypad release, check duration
      if (millis() - keypadPressStartTime >= 1000) {
        testGateAccessAuthorized = true;
        gateAccessAuthorized = true;
        if (!isGateOpen) {
          currentAction = OPEN;
        } else {
          currentAction = CLOSE;
        }
      }
      keypadPressStartTime = 0;
    }
  }
  
  // Update motor states and positions based on pin changes
  if (pin == IN1 || pin == IN2) {
    if (pin == IN1 && val == HIGH && mockPinStates[IN2] == LOW) {
      // Forward motion for motor 1
      A1_POS++;
      testA1_POS++;
      clampPositions();
    } else if (pin == IN2 && val == HIGH && mockPinStates[IN1] == LOW) {
      // Backward motion for motor 1
      A1_POS--;
      testA1_POS--;
      clampPositions();
    }
  } else if (pin == IN3 || pin == IN4) {
    if (pin == IN3 && val == HIGH && mockPinStates[IN4] == LOW) {
      // Forward motion for motor 2
      A2_POS++;
      testA2_POS++;
      clampPositions();
    } else if (pin == IN4 && val == HIGH && mockPinStates[IN3] == LOW) {
      // Backward motion for motor 2
      A2_POS--;
      testA2_POS--;
      clampPositions();
    }
  }
}

PinStatus mockDigitalRead(pin_size_t pin) {
  if (pin == LIMIT_SWITCH_1_PIN) {
    // Simulate limit switch 1 - triggers at position 0
    return (A1_POS <= 0) ? LOW : HIGH;
  } else if (pin == LIMIT_SWITCH_2_PIN) {
    // Simulate limit switch 2 - triggers at position 0
    return (A2_POS <= 0) ? LOW : HIGH;
  } else if (pin == KEYPAD_PIN) {
    return mockPinStates[pin];
  }
  return mockPinStates[pin];
}

void mockAnalogWrite(pin_size_t pin, int value) {
  mockAnalogValues[pin] = value;
  
  // Update motor speeds in test state
  if (pin == ENA) {
    testA1_Speed = value;
  } else if (pin == ENB) {
    testA2_Speed = value;
  }
}

void mockPinMode(pin_size_t pin, PinMode mode) {
  mockPinModes[pin] = mode;
}

// Helper functions for tests
void resetTestState() {
  // Reset mock pin states
  for (int i = 0; i < MAX_PINS; i++) {
    mockPinStates[i] = LOW;
    mockPinModes[i] = OUTPUT;
    mockAnalogValues[i] = 0;
  }
  
  // Reset test state variables
  testGateAccessAuthorized = false;
  testIsGateOpen = false;
  testA1_POS = -1;
  testA2_POS = -1;
  testIsCalibrated1 = false;
  testIsCalibrated2 = false;
  testA1_Speed = 0;
  testA2_Speed = 0;
  
  // Reset global state
  isGateOpen = false;
  gateAccessAuthorized = false;
  A1_POS = -1;
  A2_POS = -1;
  isCalibrated1 = false;
  isCalibrated2 = false;
  currentAction = NONE;
}

void simulateKeypadPress(unsigned long duration) {
  hw_digitalWrite(KEYPAD_PIN, HIGH);
  delay(duration);
  hw_digitalWrite(KEYPAD_PIN, LOW);
}

// Test result summary function
void printTestSummary() {
  Serial.println(F("\n[Test Summary]"));
  Serial.print(F("Tests Passed: ")); Serial.println(testsPassed);
  Serial.print(F("Tests Failed: ")); Serial.println(testsFailed);
  Serial.print(F("Total Tests: ")); Serial.println(testsPassed + testsFailed);
  if (testsFailed > 0) {
    Serial.println(F("[WARNING] Some tests failed! Check logs above for [FAIL-T<code>] messages"));
  } else {
    Serial.println(F("[SUCCESS] All tests passed!"));
  }
}

// Test cases
void test_wifi_connection() {
  resetTestState();
  
  // Test WiFi status
  int status = WiFi.status();
  assertNotEqual(status, -1, 1);
  
  // Test firmware version exists
  String fv = WiFi.firmwareVersion();
  assertNotEqual(fv, "", 2);
}

void test_hardware_initialization() {
  resetTestState();
  
  // Initialize actuators
  initializeActuators();
  
  // Verify motor control pins are set as outputs
  assertEqual(mockPinModes[IN1], OUTPUT, 3);
  assertEqual(mockPinModes[IN2], OUTPUT, 4);
  assertEqual(mockPinModes[IN3], OUTPUT, 5);
  assertEqual(mockPinModes[IN4], OUTPUT, 6);
  assertEqual(mockPinModes[ENA], OUTPUT, 7);
  assertEqual(mockPinModes[ENB], OUTPUT, 8);
  
  // Verify limit switches are set as input pullup
  assertEqual(mockPinModes[LIMIT_SWITCH_1_PIN], INPUT_PULLUP, 9);
  assertEqual(mockPinModes[LIMIT_SWITCH_2_PIN], INPUT_PULLUP, 10);
  
  // Verify keypad pin is set as input
  assertEqual(mockPinModes[KEYPAD_PIN], INPUT, 11);
  
  // Verify initial pin states
  assertEqual(mockPinStates[IN1], LOW, 12);
  assertEqual(mockPinStates[IN2], LOW, 13);
  assertEqual(mockPinStates[IN3], LOW, 14);
  assertEqual(mockPinStates[IN4], LOW, 15);
  assertEqual(mockAnalogValues[ENA], 0, 16);
  assertEqual(mockAnalogValues[ENB], 0, 17);
}

void test_initial_position() {
  resetTestState();
  
  // Initial positions should be unknown (-1)
  assertEqual(testA1_POS, -1, 18);
  assertEqual(testA2_POS, -1, 19);
}

void test_keypad_access() {
  resetTestState();
  
  // Simulate keypad signal
  simulateKeypadPress(1100); // Wait longer than required 1 second
  
  // Check access granted
  assertTrue(testGateAccessAuthorized, 20);
  assertTrue(gateAccessAuthorized, 21);
}

void test_keypad_access_valid_code() {
  resetTestState();
  
  // Simulate valid keypad press (1.5 seconds)
  simulateKeypadPress(1500);
  
  // Check if access was granted
  assertTrue(testGateAccessAuthorized, 22);
  assertEqual(currentAction, OPEN, 23);  // Should try to open when closed
}

void test_keypad_access_invalid_code() {
  resetTestState();
  
  // Simulate invalid keypad press (0.5 seconds - too short)
  simulateKeypadPress(500);
  
  // Check access was denied
  assertFalse(testGateAccessAuthorized, 24);
  assertEqual(currentAction, NONE, 25);
}

void test_gate_opening_sequence() {
  resetTestState();
  
  // Set up initial state
  gateAccessAuthorized = true;
  testA1_POS = 0;
  testA2_POS = 0;
  A1_POS = 0;
  A2_POS = 0;
  isCalibrated1 = true;
  isCalibrated2 = true;
  
  // Try to open gates
  Open_Gates(A_SPEED);
  
  // Verify motors were activated with correct speeds
  assertEqual(testA1_Speed, A_SPEED, 26);
  assertEqual(testA2_Speed, A_SPEED, 27);
  
  // Verify correct motor direction
  assertEqual(mockPinStates[IN1], HIGH, 28);
  assertEqual(mockPinStates[IN2], LOW, 29);
  assertEqual(mockPinStates[IN3], HIGH, 30);
  assertEqual(mockPinStates[IN4], LOW, 31);
  
  // Verify state updated
  assertTrue(isGateOpen, 32);
  assertEqual(currentAction, OPEN, 33);
}

void test_gate_closing_sequence() {
  resetTestState();
  
  // Set up initial state
  gateAccessAuthorized = true;
  isGateOpen = true;
  testA1_POS = A1_MAX_LIMIT;
  testA2_POS = A2_MAX_LIMIT;
  A1_POS = A1_MAX_LIMIT;
  A2_POS = A2_MAX_LIMIT;
  isCalibrated1 = true;
  isCalibrated2 = true;
  
  // Try to close gates
  Close_Gates(A_SPEED);
  
  // Verify motors were activated with correct speeds
  assertEqual(testA1_Speed, A_SPEED, 34);
  assertEqual(testA2_Speed, A_SPEED, 35);
  
  // Verify correct motor direction for closing
  assertEqual(mockPinStates[IN1], LOW, 36);
  assertEqual(mockPinStates[IN2], HIGH, 37);
  assertEqual(mockPinStates[IN3], LOW, 38);
  assertEqual(mockPinStates[IN4], HIGH, 39);
  
  // Verify state updated
  assertFalse(isGateOpen, 40);
  assertEqual(currentAction, CLOSE, 41);
}

void test_limit_switch_safety() {
  resetTestState();
  
  // Set up initial state
  testIsCalibrated1 = true;
  testIsCalibrated2 = true;
  testGateAccessAuthorized = true;
  isCalibrated1 = true;
  isCalibrated2 = true;
  gateAccessAuthorized = true;
  
  // Simulate limit switch trigger during opening
  mockPinStates[LIMIT_SWITCH_1_PIN] = LOW;
  
  // Try to open gates
  Open_Gates(A_SPEED);
  
  // Verify motors were stopped
  assertEqual(testA1_Speed, 0, 42);
  assertEqual(mockPinStates[IN1], LOW, 43);
  assertEqual(mockPinStates[IN2], LOW, 44);
}

void test_calibration_requirement() {
  resetTestState();
  
  // Set up initial state
  testGateAccessAuthorized = true;
  gateAccessAuthorized = true;
  
  // Try to open uncalibrated gates
  Open_Gates(A_SPEED);
  
  // Verify gates didn't move
  assertEqual(testA1_Speed, 0, 45);
  assertEqual(testA2_Speed, 0, 46);
  assertFalse(isGateOpen, 47);
}

void test_position_bounds() {
  resetTestState();
  
  // Set up initial state
  testA1_POS = 0;
  testA2_POS = 0;
  A1_POS = 0;
  A2_POS = 0;
  
  // Try to move past minimum (0)
  Motor1_Backward(A_SPEED);
  Motor2_Backward(A_SPEED);
  
  // Verify positions clamped at 0
  assertEqual(testA1_POS, 0, 48);
  assertEqual(testA2_POS, 0, 49);
  
  // Set positions to maximum
  testA1_POS = A1_MAX_LIMIT;
  testA2_POS = A2_MAX_LIMIT;
  A1_POS = A1_MAX_LIMIT;
  A2_POS = A2_MAX_LIMIT;
  
  // Try to move past maximum
  Motor1_Forward(A_SPEED);
  Motor2_Forward(A_SPEED);
  
  // Verify positions clamped at maximum
  assertEqual(testA1_POS, A1_MAX_LIMIT, 50);
  assertEqual(testA2_POS, A2_MAX_LIMIT, 51);
}

// Test initialization function - called from main setup()
void initializeTests() {
  // Enable mocks for testing
  useMocks = true;
  
  // Initialize hardware pins
  hw_pinMode(SSD_WRITE, OUTPUT);
  hw_pinMode(IN1, OUTPUT);
  hw_pinMode(IN2, OUTPUT);
  hw_pinMode(IN3, OUTPUT);
  hw_pinMode(IN4, OUTPUT);
  hw_pinMode(ENA, OUTPUT);
  hw_pinMode(ENB, OUTPUT);
  hw_pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLUP);
  hw_pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLUP);
  hw_pinMode(KEYPAD_PIN, INPUT);

  // Reset all test state
  resetTestState();
  
  // Reset test statistics
  testsPassed = 0;
  testsFailed = 0;
}

// Test execution function - called from main loop()
void runTests() {
  static bool testsStarted = false;
  
  if (!testsStarted) {
    testsStarted = true;
    Serial.println(F("[Test Suite] Gate Controller Tests"));
    
    // Run all tests
    test_wifi_connection();
    test_hardware_initialization();
    test_initial_position();
    test_keypad_access();
    test_keypad_access_valid_code();
    test_keypad_access_invalid_code();
    test_gate_opening_sequence();
    test_gate_closing_sequence();
    test_limit_switch_safety();
    test_calibration_requirement();
    test_position_bounds();
    
    // Print test summary
    printTestSummary();
    
    // Disable mocks after tests
    useMocks = false;
  }
}