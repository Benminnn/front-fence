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

#include <ArduinoUnit.h>
#include "front-fence-gpt.ino"
//
///////////////////////////////////////////////////////////////////////

#include <ArduinoUnit.h>
#include "front-fence-gpt.ino"

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

// Test state variables
bool testGateAccessAuthorized = false;
bool testIsGateOpen = false;
int testA1_POS = 0;
int testA2_POS = 0;
bool testIsCalibrated1 = false;
bool testIsCalibrated2 = false;

// Mock functions to simulate hardware
void mockDigitalWrite(int pin, int value) {
  mockPinStates[pin] = value;
  Serial.print("[Test] Digital Write - Pin: ");
  Serial.print(pin);
  Serial.print(" Value: ");
  Serial.println(value);
}

int mockDigitalRead(int pin) {
  int value = mockPinStates[pin];
  Serial.print("[Test] Digital Read - Pin: ");
  Serial.print(pin);
  Serial.print(" Value: ");
  Serial.println(value);
  return value;
}

void mockAnalogWrite(int pin, int value) {
  mockAnalogValues[pin] = value;
  Serial.print("[Test] Analog Write - Pin: ");
  Serial.print(pin);
  Serial.print(" Value: ");
  Serial.println(value);
}

// Test setup
void setup() {
  Serial.begin(9600);
  while(!Serial) {}
  
  Serial.println("[Test] Starting gate controller test suite...");
  
  // Replace hardware functions with mocks
  digitalWrite = mockDigitalWrite;
  digitalRead = mockDigitalRead;
  analogWrite = mockAnalogWrite;
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
  }
  
  // Reset main code variables
  gateAccessAuthorized = testGateAccessAuthorized;
  isGateOpen = testIsGateOpen;
  A1_POS = testA1_POS;
  A2_POS = testA2_POS;
  isCalibrated1 = testIsCalibrated1;
  isCalibrated2 = testIsCalibrated2;
  
  Serial.println("[Test] Test state reset complete");
}

void simulateKeypadPress(unsigned long duration) {
  Serial.print("[Test] Simulating keypad press for ");
  Serial.print(duration);
  Serial.println("ms");
  
  mockPinStates[KEYPAD_PIN] = HIGH;
  delay(duration);
  mockPinStates[KEYPAD_PIN] = LOW;
}

// Test cases

test(keypad_access_valid_code) {
  resetTestState();
  Serial.println("[Test] Running: keypad_access_valid_code");
  
  // Simulate valid keypad press (1.5 seconds)
  simulateKeypadPress(1500);
  
  // Check if access was granted
  assertTrue(gateAccessAuthorized);
  assertEqual(currentAction, OPEN);  // Should try to open when closed
}

test(keypad_access_invalid_code) {
  resetTestState();
  Serial.println("[Test] Running: keypad_access_invalid_code");
  
  // Simulate invalid keypad press (0.5 seconds - too short)
  simulateKeypadPress(500);
  
  // Check access was denied
  assertFalse(gateAccessAuthorized);
  assertEqual(currentAction, NONE);
}

test(gate_opening_sequence) {
  resetTestState();
  Serial.println("[Test] Running: gate_opening_sequence");
  
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

test(gate_closing_sequence) {
  resetTestState();
  Serial.println("[Test] Running: gate_closing_sequence");
  
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

test(limit_switch_safety) {
  resetTestState();
  Serial.println("[Test] Running: limit_switch_safety");
  
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

test(calibration_requirement) {
  resetTestState();
  Serial.println("[Test] Running: calibration_requirement");
  
  testGateAccessAuthorized = true;
  gateAccessAuthorized = true;
  
  // Try to open uncalibrated gates
  Open_Gates(A_SPEED);
  
  // Verify gates didn't move
  assertEqual(mockAnalogValues[ENA], 0);
  assertEqual(mockAnalogValues[ENB], 0);
  assertFalse(isGateOpen);
}

test(position_bounds) {
  resetTestState();
  Serial.println("[Test] Running: position_bounds");
  
  testIsCalibrated1 = true;
  testIsCalibrated2 = true;
  testGateAccessAuthorized = true;
  testA1_POS = A1_MAX_LIMIT;
  isCalibrated1 = true;
  isCalibrated2 = true;
  gateAccessAuthorized = true;
  A1_POS = A1_MAX_LIMIT;
  
  // Try to open gates when one is at limit
  Open_Gates(A_SPEED);
  
  // Verify movement was prevented
  assertEqual(mockAnalogValues[ENA], 0);
  assertTrue(A1_POS <= A1_MAX_LIMIT);
}

void loop() {
  Test::run();
}
