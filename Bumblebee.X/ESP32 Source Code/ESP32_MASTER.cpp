/**
 * File:   ESP32_MASTER.cpp
 * Author: william adebiyi
 *
 * Created on April 20, 2025
 *
 * Remote Control Master for RC Car
 * ENEL 300 - Winter 2025
 *
 * This module is the master controller for the RC car project.
 * It runs on the ESP32 in the handheld controller and manages:
 *
 * - Reading analog joystick inputs for steering and throttle
 * - Processing button presses for metal detector and headlights
 * - Establishing and maintaining Bluetooth connection with the car
 * - Transmitting control commands to the slave ESP32 on the car
 *
 * Communication Protocol:
 * Commands are sent as strings in the format "X:YYYY" where:
 * - X is a single character command type:
 *   S = Steering (0-4095)
 *   T = Throttle (0-4095)
 *   M = Metal Detector (0 = UP, 1 = DOWN)
 *   H = Headlight (0 = OFF, 1 = ON)
 * - YYYY is the value for the command
 *
 * Hardware Connections:
 * - GPIO 36: Steering joystick Y-axis (analog)
 * - GPIO 39: Throttle joystick X-axis (analog)
 * - GPIO 13: Metal detector button (digital, pull-up)
 * - GPIO 15: Headlight button (digital, pull-up)
 * - GPIO 2:  Status LED (on when connected)
 *
 * Dependencies:
 * - BluetoothSerial library for ESP32
 */

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial SerialBT;

// Replace with your SLAVE's MAC address - update from your slave's serial output
// You'll need to update this with your actual slave ESP32 MAC address
uint8_t slaveMAC[] = { 0x34, 0x5F, 0x45, 0xE7, 0x6B, 0xEA };

// Connection status LED
#define STATUS_LED 2  // Built-in LED on most ESP32 dev boards

// Joystick 1 (Steering) pins
#define STEERING_PIN 36  // GPIO 36 - Left/right steering
#define METAL_DETECTOR_BUTTON 13  // GPIO13 for metal detector control

// Joystick 2 (Throttle) pins
#define THROTTLE_PIN 39  // GPIO 39 - Forward/backward + speed
#define HEADLIGHT_BUTTON  15  // GPIO15 for headlight toggle

// Communication protocol definitions
#define CMD_STEERING 'S'
#define CMD_THROTTLE 'T'
#define CMD_METAL_DETECTOR 'M'
#define CMD_HEADLIGHT 'H'

// Button debounce variables
const unsigned long debounceDelay = 50;

// Variables for button state tracking
bool metalDetectorReading = false;
bool headlightReading = false;

bool metalDetectorState = false;
bool headlightState = false;

// Button toggle state tracking for light switch behavior
bool headlightButtonPressed = false;  // True when button is currently pressed

// Timestamps for button changes
unsigned long lastMetalDetectorChangeTime = 0;
unsigned long lastHeadlightChangeTime = 0;

// Connection status
bool isConnected = false;

// Timestamp for connection check
unsigned long lastConnectionCheck = 0;
const unsigned long connectionCheckInterval = 500; // Check connection every 500ms

void setup() {
  Serial.begin(115200);
  delay(100); // Short delay to ensure serial is ready
  
  Serial.println("\n====== RC CAR CONTROLLER (MASTER ESP32) ======");
  Serial.println("Initializing...");
  
  // Setup GPIO pins
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW); // Make sure LED starts off
  
  // Button inputs with proper pull-ups on new pins
  pinMode(METAL_DETECTOR_BUTTON, INPUT_PULLUP);  // Now on GPIO13
  pinMode(HEADLIGHT_BUTTON, INPUT_PULLUP);      // Now on GPIO15
  
  // Analog inputs
  pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);
  
  // Initial button check
  Serial.println("\nInitial button states:");
  Serial.println("Metal Detector Button: " + String(digitalRead(METAL_DETECTOR_BUTTON) == LOW ? "PRESSED" : "RELEASED"));
  Serial.println("Headlight Button: " + String(digitalRead(HEADLIGHT_BUTTON) == LOW ? "PRESSED" : "RELEASED"));
  
  // Start Bluetooth Serial
  SerialBT.begin("RC_CAR_CONTROLLER", true);  // true = master mode
  Serial.println("\nBluetooth Master mode initialized");
  Serial.println("Connecting to car (slave ESP32)...");
  
  // Blink LED to indicate ready
  blinkLED(3, 300);
  
  // Try to connect to the slave
  connectToSlave();
}

void loop() {
  // Check Bluetooth connection status more reliably
  checkConnectionStatus();
  
  // Skip the rest of the loop if not connected
  if (!isConnected) {
    return;
  }
  
  // Read and send joystick values
  processJoysticks();
  
  // Check buttons with debouncing
  checkButtons();
}

void checkConnectionStatus() {
  // Check connection periodically to save resources
  unsigned long currentTime = millis();
  if (currentTime - lastConnectionCheck < connectionCheckInterval && isConnected) {
    // Skip checking if we recently checked and are connected
    return;
  }
  
  lastConnectionCheck = currentTime;
  
  // Check if connection status has changed
  bool newConnectionStatus = false;
  
  // Multiple checks to verify we're really connected
  // The SerialBT.hasClient() is key here
  if (SerialBT.connected()) {
    if (SerialBT.hasClient()) {
      newConnectionStatus = true;
    }
  }
  
  // Handle connection state changes
  if (newConnectionStatus != isConnected) {
    isConnected = newConnectionStatus;
    
    if (isConnected) {
      // Just connected
      digitalWrite(STATUS_LED, HIGH);
      Serial.println("✅ Connected to car!");
      
      // Send initial state after connection
      delay(500); // Short delay to ensure connection is stable
      
      // Send all current states to ensure car starts in the right state
      processJoysticks();
      
      String command = String(CMD_HEADLIGHT) + ":" + (headlightState ? "1" : "0");
      SerialBT.println(command);
      
      command = String(CMD_METAL_DETECTOR) + ":" + (metalDetectorState ? "1" : "0");
      SerialBT.println(command);
    } else {
      // Just disconnected
      digitalWrite(STATUS_LED, LOW);
      Serial.println("❌ Disconnected from car. Will try to reconnect...");
      // Try to reconnect right away
      connectToSlave();
    }
  }
  
  // Always blink LED when disconnected, regardless of state change
  if (!isConnected) {
    // Blink pattern: 500ms on, 500ms off
    if (currentTime % 1000 < 500) {
      digitalWrite(STATUS_LED, HIGH);
    } else {
      digitalWrite(STATUS_LED, LOW);
    }
    
    // Try to reconnect periodically
    static unsigned long lastReconnectAttempt = 0;
    if (currentTime - lastReconnectAttempt > 5000) { // Try every 5 seconds
      lastReconnectAttempt = currentTime;
      connectToSlave();
    }
  }
}

void connectToSlave() {
  Serial.print("Attempting to connect to car at MAC address: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", slaveMAC[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  // Ensure LED is off before attempting connection
  digitalWrite(STATUS_LED, LOW);
  
  if (SerialBT.connect(slaveMAC)) {
    Serial.println("✅ Connection initiated!");
    // We don't set isConnected = true here anymore
    // We'll wait for checkConnectionStatus to confirm it
  } else {
    Serial.println("❌ Connection failed. Will retry later...");
  }
}

void processJoysticks() {
  // Read analog values
  int steeringValue = analogRead(STEERING_PIN);
  int throttleValue = analogRead(THROTTLE_PIN);
  
  // Send steering value
  String steeringCommand = String(CMD_STEERING) + ":" + String(steeringValue);
  SerialBT.println(steeringCommand);
  
  // Send throttle value
  String throttleCommand = String(CMD_THROTTLE) + ":" + String(throttleValue);
  SerialBT.println(throttleCommand);
  
  // Debug output - single line format for better readability
  // Print on EVERY loop for constant serial output
  char buffer[120];
  sprintf(buffer,
          "CTRL | STR:%4d | THR:%4d | HDL:%s | MDT:%s",
          steeringValue,
          throttleValue,
          headlightState ? "ON " : "OFF",
          metalDetectorState ? "DOWN" : "UP  ");
  
  Serial.println(buffer);
}

void checkButtons() {
  // ========== METAL DETECTOR BUTTON (GPIO13) ==========
  // Press and release behavior for metal detector
  bool currentMetalDetectorReading = (digitalRead(METAL_DETECTOR_BUTTON) == LOW);
  
  // Check if reading has changed
  if (currentMetalDetectorReading != metalDetectorReading) {
    // Reset the debounce timer
    lastMetalDetectorChangeTime = millis();
    metalDetectorReading = currentMetalDetectorReading;
  }
  
  // If the reading has been stable for longer than the debounce delay
  if ((millis() - lastMetalDetectorChangeTime) > debounceDelay) {
    // If button is pressed (LOW) and was previously not pressed
    if (metalDetectorReading && !metalDetectorState) {
      // Toggle the actual state
      metalDetectorState = true;
      // Send command
      String command = String(CMD_METAL_DETECTOR) + ":" + "1";
      SerialBT.println(command);
      Serial.println("Metal detector button pressed: DOWN");
    }
    else if (!metalDetectorReading && metalDetectorState) {
      // Button released
      metalDetectorState = false;
      String command = String(CMD_METAL_DETECTOR) + ":" + "0";
      SerialBT.println(command);
      Serial.println("Metal detector button released: UP");
    }
  }
  
  // ========== HEADLIGHT BUTTON (GPIO15) ==========
  // Modified for toggle switch behavior
  bool currentHeadlightReading = (digitalRead(HEADLIGHT_BUTTON) == LOW);
  
  // Check if reading has changed
  if (currentHeadlightReading != headlightReading) {
    // Reset the debounce timer
    lastHeadlightChangeTime = millis();
    headlightReading = currentHeadlightReading;
  }
  
  // If the reading has been stable for longer than the debounce delay
  if ((millis() - lastHeadlightChangeTime) > debounceDelay) {
    // Only toggle state on button press (not release)
    // Button is pressed down (LOW with pull-up)
    if (headlightReading && !headlightButtonPressed) {
      headlightButtonPressed = true;
      
      // Toggle the headlight state
      headlightState = !headlightState;
      
      // Send command based on new state
      String command = String(CMD_HEADLIGHT) + ":" + (headlightState ? "1" : "0");
      SerialBT.println(command);
      Serial.println("Headlight toggled: " + String(headlightState ? "ON" : "OFF"));
    }
    // Button is released (HIGH with pull-up)
    else if (!headlightReading && headlightButtonPressed) {
      headlightButtonPressed = false;
      // Do nothing on release - the toggle already happened on press
    }
  }
}

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(delayMs);
    digitalWrite(STATUS_LED, LOW);
    delay(delayMs);
  }
}
