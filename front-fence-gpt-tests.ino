///////////////////////////////////////////////////////////////////////
//
//      Gate Controller Test Suite
//      Tests for front-fence-gpt.ino
//
//      @benhodes
//      3/9/25
//      MIT License
//      All Rights Reserved
//
///////////////////////////////////////////////////////////////////////

#include <WiFiNINA.h>

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

// External references to variables we need to test
extern bool gateAccessAuthorized;
extern bool isGateOpen;
extern int A1_POS;
extern int A2_POS;
extern bool isCalibrated1;
extern bool isCalibrated2;
extern Action currentAction;

// Mock pin states
int mockPinStates[20] = {0};  // Store mock digital pin states
int mockAnalogValues[20] = {0};  // Store mock analog pin values
int mockPinModes[20] = {0};   // Store mock pin modes

// Test state variables
bool testGateAccessAuthorized = false;
bool testIsGateOpen = false;
int testA1_POS = 0;
int testA2_POS = 0;
bool testIsCalibrated1 = false;
bool testIsCalibrated2 = false;

// Flag to control whether we use mocks or real hardware
bool useMocks = false;

// Mock functions with correct types
void mockDigitalWrite(pin_size_t pin, PinStatus val) {
  mockPinStates[pin] = val;
}

PinStatus mockDigitalRead(pin_size_t pin) {
  PinStatus value = (PinStatus)mockPinStates[pin];
  return value;
}

void mockAnalogWrite(pin_size_t pin, int value) {
  mockAnalogValues[pin] = value;
}

void mockPinMode(pin_size_t pin, PinMode mode) {
  mockPinModes[pin] = mode;
}

// Wrapper functions that switch between mock and real hardware
void testDigitalWrite(pin_size_t pin, PinStatus val) {
  if (useMocks) {
    mockDigitalWrite(pin, val);
  } else {
    ::digitalWrite(pin, val);
  }
}

PinStatus testDigitalRead(pin_size_t pin) {
  if (useMocks) {
    return mockDigitalRead(pin);
  }
  return ::digitalRead(pin);
}

void testAnalogWrite(pin_size_t pin, int value) {
  if (useMocks) {
    mockAnalogWrite(pin, value);
  } else {
    ::analogWrite(pin, value);
  }
}

void testPinMode(pin_size_t pin, PinMode mode) {
  if (useMocks) {
    mockPinMode(pin, mode);
  } else {
    ::pinMode(pin, mode);
  }
}

// Helper functions for tests
void resetTestState() {
  testGateAccessAuthorized = false;
  testIsGateOpen = false;
  testA1_POS = 0;
  testA2_POS = 0;
  testIsCalibrated1 = false;
  testIsCalibrated2 = false;
  currentAction = NONE;
  
  // Reset mock hardware state
  for(int i = 0; i < 20; i++) {
    mockPinStates[i] = 0;
    mockAnalogValues[i] = 0;
    mockPinModes[i] = 0;
  }
  
  // Reset main code variables
  gateAccessAuthorized = testGateAccessAuthorized;
  isGateOpen = testIsGateOpen;
  A1_POS = testA1_POS;
  A2_POS = testA2_POS;
  isCalibrated1 = testIsCalibrated1;
  isCalibrated2 = testIsCalibrated2;
}

void simulateKeypadPress(unsigned long duration) {
  mockPinStates[KEYPAD_PIN] = HIGH;
  delay(duration);
  mockPinStates[KEYPAD_PIN] = LOW;
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
  
  // Verify initial positions are unknown
  assertEqual(A1_POS, -1, 18);
  assertEqual(A2_POS, -1, 19);
}

void test_keypad_access() {
  resetTestState();
  
  // Test valid keypad press
  simulateKeypadPress(1500); // 1.5 seconds
  assertTrue(gateAccessAuthorized, 20);
  
  // Test invalid keypad press
  resetTestState();
  simulateKeypadPress(500); // 0.5 seconds (too short)
  assertFalse(gateAccessAuthorized, 21);
}

void test_keypad_access_valid_code() {
  resetTestState();
  
  // Simulate valid keypad press (1.5 seconds)
  simulateKeypadPress(1500);
  
  // Check if access was granted
  assertTrue(gateAccessAuthorized, 22);
  assertEqual(currentAction, OPEN, 23);  // Should try to open when closed
}

void test_keypad_access_invalid_code() {
  resetTestState();
  
  // Simulate invalid keypad press (0.5 seconds - too short)
  simulateKeypadPress(500);
  
  // Check access was denied
  assertFalse(gateAccessAuthorized, 24);
  assertEqual(currentAction, NONE, 25);
}

void test_gate_opening_sequence() {
  resetTestState();
  
  testIsCalibrated1 = true;
  testIsCalibrated2 = true;
  testGateAccessAuthorized = true;
  gateAccessAuthorized = true;
  isCalibrated1 = true;
  isCalibrated2 = true;
  
  // Try to open gates
  Open_Gates(A_SPEED);
  
  // Verify motors were activated
  assertEqual(mockAnalogValues[ENA], A1_SPEED, 26);
  assertEqual(mockAnalogValues[ENB], A2_SPEED, 27);
  assertEqual(mockPinStates[IN1], HIGH, 28);
  assertEqual(mockPinStates[IN2], LOW, 29);
  assertEqual(mockPinStates[IN3], HIGH, 30);
  assertEqual(mockPinStates[IN4], LOW, 31);
  
  // Verify state changes
  assertTrue(isGateOpen, 32);
  assertFalse(gateAccessAuthorized, 33);  // Should reset after opening
}

void test_gate_closing_sequence() {
  resetTestState();
  
  testA1_POS = A1_MAX_LIMIT;
  testA2_POS = A2_MAX_LIMIT;
  testIsGateOpen = true;
  A1_POS = A1_MAX_LIMIT;
  A2_POS = A2_MAX_LIMIT;
  isGateOpen = true;
  
  // Try to close gates
  Close_Gates(A_SPEED);
  
  // Verify motors were activated in reverse
  assertEqual(mockAnalogValues[ENA], A1_SPEED, 34);
  assertEqual(mockAnalogValues[ENB], A2_SPEED, 35);
  assertEqual(mockPinStates[IN1], LOW, 36);
  assertEqual(mockPinStates[IN2], HIGH, 37);
  assertEqual(mockPinStates[IN3], LOW, 38);
  assertEqual(mockPinStates[IN4], HIGH, 39);
  
  // Verify state changes
  assertFalse(isGateOpen, 40);
  assertEqual(A1_POS, 0, 41);
  assertEqual(A2_POS, 0, 42);
}

void test_limit_switch_safety() {
  resetTestState();
  
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
  assertEqual(mockAnalogValues[ENA], 0, 43);
  assertEqual(mockPinStates[IN1], LOW, 44);
  assertEqual(mockPinStates[IN2], LOW, 45);
}

void test_calibration_requirement() {
  resetTestState();
  
  testGateAccessAuthorized = true;
  gateAccessAuthorized = true;
  
  // Try to open uncalibrated gates
  Open_Gates(A_SPEED);
  
  // Verify gates didn't move
  assertEqual(mockAnalogValues[ENA], 0, 46);
  assertEqual(mockAnalogValues[ENB], 0, 47);
  assertFalse(isGateOpen, 48);
}

void test_position_bounds() {
  resetTestState();
  
  // Test upper bounds
  A1_POS = A1_MAX_LIMIT + 10;
  A2_POS = A2_MAX_LIMIT + 10;
  
  // Should clamp to max
  assertEqual(A1_POS, A1_MAX_LIMIT, 49);
  assertEqual(A2_POS, A2_MAX_LIMIT, 50);
  
  // Test lower bounds
  A1_POS = -10;
  A2_POS = -10;
  
  // Should clamp to 0
  assertEqual(A1_POS, 0, 51);
  assertEqual(A2_POS, 0, 52);
}

// Test initialization function - called from main setup()
void initializeTests() {
  testsPassed = 0;
  testsFailed = 0;
}

// Test execution function - called from main loop()
void runTests() {
  static bool testsStarted = false;
  
  if (!testsStarted) {
    testsStarted = true;
    Serial.println(F("[Test Suite] Gate Controller Tests"));
    
    // Enable mocks for testing
    useMocks = true;
    
    // Run all tests silently
    test_wifi_connection();
    test_hardware_initialization();
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
