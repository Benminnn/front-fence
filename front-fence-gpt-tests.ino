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

// Simple test framework
#define assertEqual(a, b) do { \
  if ((a) != (b)) { \
    Serial.print(F("[FAIL] Expected ")); \
    Serial.print(a); \
    Serial.print(F(" to equal ")); \
    Serial.println(b); \
    testsFailed++; \
  } else { \
    Serial.println(F("[PASS] Assertion passed")); \
    testsPassed++; \
  } \
} while(0)

#define assertNotEqual(a, b) do { \
  if ((a) == (b)) { \
    Serial.print(F("[FAIL] Expected ")); \
    Serial.print(a); \
    Serial.print(F(" to not equal ")); \
    Serial.println(b); \
    testsFailed++; \
  } else { \
    Serial.println(F("[PASS] Assertion passed")); \
    testsPassed++; \
  } \
} while(0)

#define assertTrue(x) assertEqual((x), true)
#define assertFalse(x) assertEqual((x), false)

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
  Serial.print(F("[Test] Digital Write - Pin: "));
  Serial.print(pin);
  Serial.print(F(" Value: "));
  Serial.println(val);
}

PinStatus mockDigitalRead(pin_size_t pin) {
  PinStatus value = (PinStatus)mockPinStates[pin];
  Serial.print(F("[Test] Digital Read - Pin: "));
  Serial.print(pin);
  Serial.print(F(" Value: "));
  Serial.println(value);
  return value;
}

void mockAnalogWrite(pin_size_t pin, int value) {
  mockAnalogValues[pin] = value;
  Serial.print(F("[Test] Analog Write - Pin: "));
  Serial.print(pin);
  Serial.print(F(" Value: "));
  Serial.println(value);
}

void mockPinMode(pin_size_t pin, PinMode mode) {
  mockPinModes[pin] = mode;
  Serial.print(F("[Test] Pin Mode - Pin: "));
  Serial.print(pin);
  Serial.print(F(" Mode: "));
  Serial.println(mode == OUTPUT ? F("OUTPUT") : mode == INPUT ? F("INPUT") : F("INPUT_PULLUP"));
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
  
  Serial.println(F("[Test] Test state reset complete"));
}

void simulateKeypadPress(unsigned long duration) {
  Serial.print(F("[Test] Simulating keypad press for "));
  Serial.print(duration);
  Serial.println(F("ms"));
  
  mockPinStates[KEYPAD_PIN] = HIGH;
  delay(duration);
  mockPinStates[KEYPAD_PIN] = LOW;
}

// Test cases
void test_wifi_connection() {
  resetTestState();
  Serial.println(F("[Test] Running: wifi_connection"));
  
  // Test WiFi status
  int status = WiFi.status();
  assertNotEqual(status, -1); // Basic check that WiFi module responds
  
  // Test firmware version exists
  String fv = WiFi.firmwareVersion();
  assertNotEqual(fv, "");
}

void test_hardware_initialization() {
  resetTestState();
  Serial.println(F("[Test] Running: hardware_initialization"));
  
  // Initialize actuators
  initializeActuators();
  
  // Verify motor control pins are set as outputs
  assertEqual(mockPinModes[IN1], OUTPUT);
  assertEqual(mockPinModes[IN2], OUTPUT);
  assertEqual(mockPinModes[IN3], OUTPUT);
  assertEqual(mockPinModes[IN4], OUTPUT);
  assertEqual(mockPinModes[ENA], OUTPUT);
  assertEqual(mockPinModes[ENB], OUTPUT);
  
  // Verify limit switches are set as input pullup
  assertEqual(mockPinModes[LIMIT_SWITCH_1_PIN], INPUT_PULLUP);
  assertEqual(mockPinModes[LIMIT_SWITCH_2_PIN], INPUT_PULLUP);
  
  // Verify keypad pin is set as input
  assertEqual(mockPinModes[KEYPAD_PIN], INPUT);
  
  // Verify initial pin states
  assertEqual(mockPinStates[IN1], LOW);
  assertEqual(mockPinStates[IN2], LOW);
  assertEqual(mockPinStates[IN3], LOW);
  assertEqual(mockPinStates[IN4], LOW);
  assertEqual(mockAnalogValues[ENA], 0);
  assertEqual(mockAnalogValues[ENB], 0);
  
  // Verify initial positions are unknown
  assertEqual(A1_POS, -1);
  assertEqual(A2_POS, -1);
}

void test_keypad_access() {
  resetTestState();
  Serial.println(F("[Test] Running: keypad_access"));
  
  // Test valid keypad press
  simulateKeypadPress(1500); // 1.5 seconds
  assertTrue(gateAccessAuthorized);
  
  // Test invalid keypad press
  resetTestState();
  simulateKeypadPress(500); // 0.5 seconds (too short)
  assertFalse(gateAccessAuthorized);
}

void test_keypad_access_valid_code() {
  resetTestState();
  Serial.println(F("[Test] Running: keypad_access_valid_code"));
  
  // Simulate valid keypad press (1.5 seconds)
  simulateKeypadPress(1500);
  
  // Check if access was granted
  assertTrue(gateAccessAuthorized);
  assertEqual(currentAction, OPEN);  // Should try to open when closed
}

void test_keypad_access_invalid_code() {
  resetTestState();
  Serial.println(F("[Test] Running: keypad_access_invalid_code"));
  
  // Simulate invalid keypad press (0.5 seconds - too short)
  simulateKeypadPress(500);
  
  // Check access was denied
  assertFalse(gateAccessAuthorized);
  assertEqual(currentAction, NONE);
}

void test_gate_opening_sequence() {
  resetTestState();
  Serial.println(F("[Test] Running: gate_opening_sequence"));
  
  testIsCalibrated1 = true;
  testIsCalibrated2 = true;
  testGateAccessAuthorized = true;
  gateAccessAuthorized = true;
  isCalibrated1 = true;
  isCalibrated2 = true;
  
  // Try to open gates
  Open_Gates(A_SPEED);
  
  // Verify motors were activated
  assertEqual(mockAnalogValues[ENA], A1_SPEED);
  assertEqual(mockAnalogValues[ENB], A2_SPEED);
  assertEqual(mockPinStates[IN1], HIGH);
  assertEqual(mockPinStates[IN2], LOW);
  assertEqual(mockPinStates[IN3], HIGH);
  assertEqual(mockPinStates[IN4], LOW);
  
  // Verify state changes
  assertTrue(isGateOpen);
  assertFalse(gateAccessAuthorized);  // Should reset after opening
}

void test_gate_closing_sequence() {
  resetTestState();
  Serial.println(F("[Test] Running: gate_closing_sequence"));
  
  testA1_POS = A1_MAX_LIMIT;
  testA2_POS = A2_MAX_LIMIT;
  testIsGateOpen = true;
  A1_POS = A1_MAX_LIMIT;
  A2_POS = A2_MAX_LIMIT;
  isGateOpen = true;
  
  // Try to close gates
  Close_Gates(A_SPEED);
  
  // Verify motors were activated in reverse
  assertEqual(mockAnalogValues[ENA], A1_SPEED);
  assertEqual(mockAnalogValues[ENB], A2_SPEED);
  assertEqual(mockPinStates[IN1], LOW);
  assertEqual(mockPinStates[IN2], HIGH);
  assertEqual(mockPinStates[IN3], LOW);
  assertEqual(mockPinStates[IN4], HIGH);
  
  // Verify state changes
  assertFalse(isGateOpen);
  assertEqual(A1_POS, 0);
  assertEqual(A2_POS, 0);
}

void test_limit_switch_safety() {
  resetTestState();
  Serial.println(F("[Test] Running: limit_switch_safety"));
  
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
  assertEqual(mockAnalogValues[ENA], 0);
  assertEqual(mockPinStates[IN1], LOW);
  assertEqual(mockPinStates[IN2], LOW);
}

void test_calibration_requirement() {
  resetTestState();
  Serial.println(F("[Test] Running: calibration_requirement"));
  
  testGateAccessAuthorized = true;
  gateAccessAuthorized = true;
  
  // Try to open uncalibrated gates
  Open_Gates(A_SPEED);
  
  // Verify gates didn't move
  assertEqual(mockAnalogValues[ENA], 0);
  assertEqual(mockAnalogValues[ENB], 0);
  assertFalse(isGateOpen);
}

void test_position_bounds() {
  resetTestState();
  Serial.println(F("[Test] Running: position_bounds"));
  
  // Test upper bounds
  A1_POS = A1_MAX_LIMIT + 10;
  A2_POS = A2_MAX_LIMIT + 10;
  
  // Should clamp to max
  assertEqual(A1_POS, A1_MAX_LIMIT);
  assertEqual(A2_POS, A2_MAX_LIMIT);
  
  // Test lower bounds
  A1_POS = -10;
  A2_POS = -10;
  
  // Should clamp to 0
  assertEqual(A1_POS, 0);
  assertEqual(A2_POS, 0);
}

// Test initialization function - called from main setup()
void initializeTests() {
  Serial.println(F("[Test] Initializing test suite..."));
  testsPassed = 0;
  testsFailed = 0;
}

// Test execution function - called from main loop()
void runTests() {
  static bool testsStarted = false;
  
  if (!testsStarted) {
    testsStarted = true;
    Serial.println(F("[Test] Starting test execution..."));
    
    // Enable mocks for testing
    useMocks = true;
    
    // Run all tests
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
    
    // Print test results
    Serial.println(F("\n[Test] Test Results:"));
    Serial.print(F("Passed: "));
    Serial.println(testsPassed);
    Serial.print(F("Failed: "));
    Serial.println(testsFailed);
    
    // Disable mocks after tests
    useMocks = false;
  }
}
