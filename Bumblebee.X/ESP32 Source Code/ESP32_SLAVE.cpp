/**
 * File:   ESP32_SLAVE.cpp
 * Author: william adebiyi
 *
 * Created on April 20, 2025
 * 
 * Slave Controller for RC Car
 * ENEL 300 - Winter 2025
 * 
 * This module is the slave controller for the RC car project.
 * It runs on the ESP32 mounted on the car and acts as a bridge between
 * the Bluetooth communication and the two AVR microcontrollers.
 * 
 * Main responsibilities:
 * - Receiving commands from the master controller via Bluetooth
 * - Converting and forwarding commands to appropriate AVR pins
 * - Maintaining safe states when connection is lost
 * - Providing status information in serial monitor
 * 
 * Outputs to AVRs:
 * DAC outputs (0-3.3V):
 * - GPIO 25: Steering control to Main AVR (PD2)
 * - GPIO 26: Motor control to Main AVR (PD3)
 * 
 * Digital outputs (HIGH/LOW):
 * - GPIO 27: Headlight toggle to Main AVR (PD5)
 * - GPIO 33: Metal detector arm toggle to Secondary AVR (PD2)
 * 
 * Status indicators:
 * - GPIO 15: Status LED (HIGH when connected, blinking when disconnected)
 * 
 * Communication Protocol:
 * Receives commands in format "X:YYYY" where:
 * - X is command type (S=Steering, T=Throttle, M=Metal Detector, H=Headlight)
 * - YYYY is the value for that command
 * 
 * Dependencies:
 * - BluetoothSerial library for ESP32
 * - esp_bt_device.h for obtaining device MAC address
 */

#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial SerialBT;

// Connection status LED
#define STATUS_LED 15  // Using GPIO15 for connection status LED

// Pins for communicating with AVR
#define STEERING_PIN 25      // GPIO 25 to AVR PD2 - Steering servo control
#define THROTTLE_PIN 26      // GPIO 26 to AVR PD3 - Motor driver control
#define METAL_DETECT_PIN 33  // GPIO 33 to AVR PD4 - Metal detector servo
#define HEADLIGHT_PIN 27     // GPIO 27 to AVR PD5 - Headlights control

// Communication protocol definitions
#define CMD_STEERING 'S'
#define CMD_THROTTLE 'T'
#define CMD_METAL_DETECTOR 'M'
#define CMD_HEADLIGHT 'H'

// Variables to track button states
bool headlightState = false;
bool metalDetectorState = false;

// Variables to track analog values for continuous display
int currentSteeringValue = 127; // Middle position (0-255)
int currentThrottleValue = 127; // Middle position (0-255)

// Raw values received from controller
int rawSteeringValue = 2048;    // Middle position (0-4095)
int rawThrottleValue = 2048;    // Middle position (0-4095)

// Improved joystick calibration
// These values need to be adjusted based on actual joystick center readings
const int JOYSTICK_STEERING_CENTER = 1930; // Adjust based on your joystick
const int JOYSTICK_THROTTLE_CENTER = 1930; // Adjust based on your joystick
const int JOYSTICK_DEADZONE = 50;         // Deadzone around center

// Status reporting
unsigned long lastStatusReport = 0;
const unsigned long statusReportInterval = 100; // More frequent status reports

// Connection status
bool isConnected = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n====== RC CAR (SLAVE ESP32) ======");
  Serial.println("Initializing...");
  
  // Setup GPIO pins
  pinMode(STATUS_LED, OUTPUT);
  pinMode(STEERING_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(METAL_DETECT_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);
  
  // Set initial states
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(METAL_DETECT_PIN, LOW);
  digitalWrite(HEADLIGHT_PIN, LOW);
  
  // Default middle values for servo and motor (127 = neutral)
  dacWrite(STEERING_PIN, 127); // Steering centered
  dacWrite(THROTTLE_PIN, 127); // Motor stopped
  
  // Start Bluetooth Serial
  SerialBT.begin("RC_CAR");
  Serial.println("Bluetooth Slave mode initialized, name: RC_CAR");
  
  // Show MAC address for pairing
  printMacAddress();

  // Blink LED to indicate ready
  blinkLED(3, 300);
}

void loop() {
  // Check connection status
  if (SerialBT.connected() != isConnected) {
    isConnected = SerialBT.connected();
    if (isConnected) {
      Serial.println("? Controller connected!");
      digitalWrite(STATUS_LED, HIGH); // LED on when connected
    } else {
      Serial.println("? Controller disconnected");
      digitalWrite(STATUS_LED, LOW); // LED off when disconnected
      
      // Reset car to safe state when disconnected
      dacWrite(STEERING_PIN, 127); // Steering centered
      dacWrite(THROTTLE_PIN, 127); // Motor stopped
      digitalWrite(HEADLIGHT_PIN, LOW); // Headlights off
      digitalWrite(METAL_DETECT_PIN, LOW); // Metal detector up
      
      // Reset tracking variables
      currentSteeringValue = 127;
      currentThrottleValue = 127;
      rawSteeringValue = 2048;
      rawThrottleValue = 2048;
    }
  }
  
  // Blink status LED when disconnected
  if (!isConnected) {
    if (millis() % 1000 < 500) {
      digitalWrite(STATUS_LED, HIGH);
    } else {
      digitalWrite(STATUS_LED, LOW);
    }
  }
  
  // Process incoming commands
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim();
    processCommand(incoming);
  }
  
  // Print comprehensive status report for continuous output
  if (millis() - lastStatusReport > statusReportInterval) {
    lastStatusReport = millis();
    printStatusReport();
  }
}

void printStatusReport() {
  // Create status report showing all values sent to AVR
  // Continuously output status in a clean format
  char buffer[120];
  
  sprintf(buffer, 
          "CAR | STR:%3d | THR:%3d | MDT:%s | HDL:%s", 
          currentSteeringValue, 
          currentThrottleValue, 
          metalDetectorState ? "DOWN" : "UP  ", 
          headlightState ? "ON " : "OFF");
  
  Serial.println(buffer);
}

void processCommand(String command) {
  // Command format: "X:YYYY" where X is the command type and YYYY is the value
  if (command.length() < 3 || command.indexOf(':') == -1) {
    Serial.println("Invalid command format: " + command);
    return;
  }
  
  char cmd = command.charAt(0);
  String value = command.substring(2);
  
  // Only print interesting commands, not the continuous joystick updates
  if (cmd != CMD_STEERING && cmd != CMD_THROTTLE) {
    char cmdBuffer[80];
    sprintf(cmdBuffer, "CMD | Type: %c | Value: %s", cmd, value.c_str());
    Serial.println(cmdBuffer);
  }
  
  switch (cmd) {
    case CMD_STEERING:
      processSteeringCommand(value);
      break;
      
    case CMD_THROTTLE:
      processThrottleCommand(value);
      break;
      
    case CMD_METAL_DETECTOR:
      processMetalDetectorCommand(value);
      break;
      
    case CMD_HEADLIGHT:
      processHeadlightCommand(value);
      break;
      
    default:
      Serial.println("Unknown command: " + command);
      break;
  }
}

void processSteeringCommand(String value) {
  // Store raw value
  rawSteeringValue = value.toInt();
  
  // Improved mapping from joystick range to servo control range
  // Use the calibrated center value instead of an assumed center
  int servoValue;
  
  // Apply deadzone around center
  if (abs(rawSteeringValue - JOYSTICK_STEERING_CENTER) < JOYSTICK_DEADZONE) {
    servoValue = 127; // Center position
  } 
  // Map left half of joystick range
  else if (rawSteeringValue < JOYSTICK_STEERING_CENTER) {
    servoValue = map(rawSteeringValue, 
                    0, JOYSTICK_STEERING_CENTER - JOYSTICK_DEADZONE, 
                    0, 127);
  } 
  // Map right half of joystick range
  else {
    servoValue = map(rawSteeringValue, 
                    JOYSTICK_STEERING_CENTER + JOYSTICK_DEADZONE, 4095, 
                    127, 255);
  }
  
  // Constrain to valid range
  servoValue = constrain(servoValue, 0, 255);
  
  // Store current mapped value
  currentSteeringValue = servoValue;
  
  // Set DAC output for steering
  dacWrite(STEERING_PIN, servoValue);
}

void processThrottleCommand(String value) {
  // Store raw value
  rawThrottleValue = value.toInt();
  
  // Improved mapping for throttle
  // Using calibrated center value
  int motorValue;
  
  // Apply deadzone around center
  if (abs(rawThrottleValue - JOYSTICK_THROTTLE_CENTER) < JOYSTICK_DEADZONE) {
    motorValue = 127; // Center position (stop)
  }
  // Map bottom half of joystick range (reverse)
  else if (rawThrottleValue < JOYSTICK_THROTTLE_CENTER) {
    motorValue = map(rawThrottleValue, 
                   0, JOYSTICK_THROTTLE_CENTER - JOYSTICK_DEADZONE, 
                   0, 127);
  }
  // Map top half of joystick range (forward)
  else {
    motorValue = map(rawThrottleValue, 
                   JOYSTICK_THROTTLE_CENTER + JOYSTICK_DEADZONE, 4095, 
                   127, 255);
  }
  
  // Constrain to valid range
  motorValue = constrain(motorValue, 0, 255);
  
  // Store current mapped value
  currentThrottleValue = motorValue;
  
  // Set DAC output for motor
  dacWrite(THROTTLE_PIN, motorValue);
}

void processMetalDetectorCommand(String value) {
  // Update metal detector state (0 = UP, 1 = DOWN)
  metalDetectorState = (value == "1");
  
  // Set pin output for metal detector servo
  digitalWrite(METAL_DETECT_PIN, metalDetectorState ? HIGH : LOW);
  
  // Print information about the metal detector command
  Serial.println("METAL | State: " + String(metalDetectorState ? "DOWN" : "UP"));
}

void processHeadlightCommand(String value) {
  // Update headlight state (0 = OFF, 1 = ON)
  headlightState = (value == "1");
  
  // Set pin output for headlights
  digitalWrite(HEADLIGHT_PIN, headlightState ? HIGH : LOW);
  
  // Print information about the headlight command
  Serial.println("LIGHTS | State: " + String(headlightState ? "ON" : "OFF"));
}

void printMacAddress() {
  const uint8_t* mac = esp_bt_dev_get_address();
  Serial.print("MAC Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.println("Copy this MAC address to the master ESP32 code");
}

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(delayMs);
    digitalWrite(STATUS_LED, LOW);
    delay(delayMs);
  }
}
