/**
 * main.c - Main AVR128DB28 for RC Car Project (Improved Motor Control)
 * ENEL 300 - Winter 2025
 * 
 * This microcontroller handles:
 * - Steering servo (PA0)
 * - DC motor control via L298N (PA1-PA3)
 * - Headlights (PA5)
 * 
 * Receives control signals from ESP32:
 * - Steering (PD2) - DAC
 * - Motor (PD3) - DAC
 * - Headlight Toggle (PD5) - Digital
 */

#define F_CPU 4000000UL  // 4MHz clock

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h> // For abs() function

//======== PIN DEFINITIONS ========//

// Steering Servo and Motor Control (TCA0)
#define SERVO_PIN 0           // PA0 (TCA0 WO0) - Steering servo
#define MOTOR_ENA_PIN 1       // PA1 (TCA0 WO1) - Motor speed control (ENA)
#define MOTOR_IN1_PIN 2       // PA2 - Motor direction control
#define MOTOR_IN2_PIN 3       // PA3 - Motor direction control

// Headlight
#define HEADLIGHT_PIN 5       // PA5 - White LED headlights

// ESP32 Communication Pins
#define STEERING_INPUT_PIN 2  // PD2 - Steering control from ESP32 (DAC)
#define MOTOR_INPUT_PIN 3     // PD3 - Motor control from ESP32 (DAC)
#define HEADLIGHT_INPUT_PIN 5 // PD5 - Headlight toggle from ESP32 (Digital)

//======== CONSTANTS & SETTINGS ========//

//===================== SERVO CONTROL (STEERING) =====================//

// Pulse width values in TCA0 ticks (at 4MHz clock, 64 prescaler ? 62.5kHz tick rate)
// 62.5kHz ? 1 tick ? 16?s ? 1.5ms pulse ? 94 ticks

#define SERVO_CENTER 94       // Neutral/center position (1.5ms pulse) [usually 94]
#define SERVO_MIN    19       // Full left (~0.3ms pulse) [19?30 depending on servo range]
#define SERVO_MAX    200      // Full right (~2.7ms pulse) [180?200 if your servo allows it]
#define SERVO_DEADBAND 5      // How much joystick wiggle is ignored near center [3?10] 


// Auto-calculated ranges for mapping
#define SERVO_LEFT_RANGE  (SERVO_CENTER - SERVO_MIN)   // Left turn distance
#define SERVO_RIGHT_RANGE (SERVO_MAX - SERVO_CENTER)   // Right turn distance


//===================== MOTOR CONTROL (THROTTLE) =====================//

#define THROTTLE_IDLE 80       // Joystick value (0?255) that represents "stop" [75?85]
#define THROTTLE_DEADBAND 10   // +/- range around idle considered "stopped" [10?20]

  // Increase THROTTLE_DEADBAND if motor jitters when idle.
  // Decrease it if motor doesn?t respond fast enough after neutral.

//============== PWM DUTY LIMITS (FORWARD / REVERSE POWER) ==============//

#define MOTOR_MIN_POWER 120    // Minimum PWM to overcome inertia (motor starts moving) [100?140]
#define MOTOR_MAX_POWER 255    // Max speed cap (full throttle) [200?255]
#define MOTOR_REV_POWER 200    // Max reverse power (separate in case you want reverse to be slower)


  // Increase MOTOR_MIN_POWER if motor "buzzes" but doesn't move.
  // Decrease MOTOR_MIN_POWER if it jumps too hard from zero.

//============== RAMP-UP / DIRECTION CHANGE SETTINGS ==============//

#define MOTOR_ACCEL_RATE 20    // How fast motor speeds up (larger = faster) [10?30]
#define MOTOR_DECEL_RATE 30    // How fast motor slows down (larger = faster) [10?30]
#define DIR_CHANGE_DELAY 10    // Delay (loop cycles) to stop before changing direction [5?15]

/*
  ? ACCEL_RATE = snappier acceleration, ? = smoother
  ? DECEL_RATE = quicker braking
  ? DIR_CHANGE_DELAY = longer pause when switching FWD ? REV (avoids hard flips)
*/


//======== VARIABLES ========//

// Input values
uint16_t steering_raw = 2048;     // Raw ADC value for steering (12-bit, centered at ~2048)
uint16_t motor_raw = 2048;        // Raw ADC value for motor (12-bit, centered at ~2048)

// Processed values
uint8_t steering_value = 127;     // Mapped steering value (8-bit, 0-255)
uint8_t motor_value = THROTTLE_IDLE; // Mapped motor value (8-bit)
bool headlight_on = false;        // Headlight state

// Motor control state
uint8_t current_duty = 0;         // Current PWM duty cycle
uint8_t target_duty = 0;          // Target PWM duty cycle
bool is_forward = true;           // Direction flag
bool was_forward = true;          // Previous direction flag
uint8_t direction_change_counter = 0; // Counter for direction change delay

// Filter buffers for analog inputs
#define FILTER_SIZE 8             // Increased to 8 as in working code
uint16_t steering_buffer[FILTER_SIZE];
uint16_t motor_buffer[FILTER_SIZE];
uint8_t buffer_index = 0;

// Timestamp for activity monitoring (simple counter)
uint32_t last_activity_time = 0;
uint32_t current_time = 0;
#define ACTIVITY_TIMEOUT 1000  // ~10 seconds at 100Hz loop

//======== FUNCTION PROTOTYPES ========//

// Initialization functions
void init_clock(void);
void init_pins(void);
void init_adc(void);
void init_pwm_tca0(void);  // For steering servo and motor

// Control functions
void control_steering(uint8_t joystick_value);
void control_motor(uint8_t joystick_value);
void control_headlight(bool on);

// Input processing
uint16_t read_adc(uint8_t pin);
uint8_t get_filtered_value(uint16_t *buffer);
bool is_signal_active(void);
void set_safe_state(void);

//======== INITIALIZATION FUNCTIONS ========//

void init_clock(void) {
    // Configure for 4MHz using internal oscillator
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;        // Select internal high-frequency oscillator
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;  // 4x prescaler (16MHz/4 = 4MHz)
    
    // Wait for clock to stabilize
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm);
}

void init_pins(void) {
    // PORTA - Configure outputs for servo, motor, headlight
    PORTA.DIRSET = (1 << SERVO_PIN) | (1 << MOTOR_ENA_PIN) | 
                   (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN) |
                   (1 << HEADLIGHT_PIN);
    
    // Set initial states
    
    // Headlight OFF initially
    PORTA.OUTCLR = (1 << HEADLIGHT_PIN);
    
    // Initialize motor direction pins for neutral state (both LOW)
    PORTA.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    
    // PORTD - Configure inputs from ESP32
    PORTD.DIRCLR = (1 << STEERING_INPUT_PIN) | (1 << MOTOR_INPUT_PIN) | 
                   (1 << HEADLIGHT_INPUT_PIN);
    
    // Initialize filter buffers
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        steering_buffer[i] = 2048;  // Center position initially
        motor_buffer[i] = 2048;     // Center position initially
    }
}

void init_adc(void) {
    // Configure ADC for 12-bit resolution with VDD reference
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;       // Use VDD (5V) as reference
    ADC0.CTRLC = ADC_PRESC_DIV4_gc;          // ADC clock prescaler (1MHz at 4MHz CPU)
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_12BIT_gc;  // Enable ADC, 12-bit resolution
}

void init_pwm_tca0(void) {
    // Configure TCA0 for PWM generation on PA0 (servo) and PA1 (motor)
    
    // Route TCA0 outputs to PORTA (this is the default, but we're explicit)
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
    
    // For a 50Hz PWM with 4MHz clock and DIV64 prescaler:
    // Tick frequency = 4MHz/64 = 62.5kHz
    // For 50Hz output, period = 62.5kHz/50Hz = 1250 ticks
    TCA0.SINGLE.PER = 1250;
    
    // Enable compare channels 0 and 1 (WO0 for servo, WO1 for motor)
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | 
                       TCA_SINGLE_CMP0EN_bm | 
                       TCA_SINGLE_CMP1EN_bm;
    
    // Center the servo initially (~1.5ms pulse = 94 ticks)
    TCA0.SINGLE.CMP0 = SERVO_CENTER;
    
    // Set motor to stopped initially
    TCA0.SINGLE.CMP1 = 0;
    
    // Start TCA0 with DIV64 prescaler
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
}

//======== CONTROL FUNCTIONS ========//

void control_steering(uint8_t joystick_value) {
    static uint8_t prev_pulse_width = SERVO_CENTER;  // For smoothing
    uint8_t target_pulse_width;
    
    // Check for center deadband
    if (joystick_value >= (127 - SERVO_DEADBAND) && 
        joystick_value <= (127 + SERVO_DEADBAND)) {
        target_pulse_width = SERVO_CENTER;
    }
    // Map joystick to servo range with improved symmetry
    else if (joystick_value < 127 - SERVO_DEADBAND) {
        // Left turn - map from 0->127-deadband to MIN->CENTER, using left range
        uint8_t range = 127 - SERVO_DEADBAND;
        uint16_t scaled = range - joystick_value;  // Invert so 0 is full left
        target_pulse_width = SERVO_CENTER - ((scaled * SERVO_LEFT_RANGE) / range);
    } 
    else {
        // Right turn - map from 127+deadband->255 to CENTER->MAX, using right range
        uint8_t range = 255 - (127 + SERVO_DEADBAND);
        uint16_t scaled = joystick_value - (127 + SERVO_DEADBAND);
        target_pulse_width = SERVO_CENTER + ((scaled * SERVO_RIGHT_RANGE) / range);
    }
    
    // Constrain pulse width to valid range
    if (target_pulse_width < SERVO_MIN) target_pulse_width = SERVO_MIN;
    if (target_pulse_width > SERVO_MAX) target_pulse_width = SERVO_MAX;
    
    // Apply smoothing to reduce jitter (move 25% of the way to target per update)
    uint8_t pulse_width = prev_pulse_width + ((target_pulse_width - prev_pulse_width) >> 2);
    prev_pulse_width = pulse_width;
    
    // Set servo position
    TCA0.SINGLE.CMP0 = pulse_width;
}

void control_motor(uint8_t joystick_value) {
    // Immediate deadzone check
    if (joystick_value >= (THROTTLE_IDLE - THROTTLE_DEADBAND) &&
        joystick_value <= (THROTTLE_IDLE + THROTTLE_DEADBAND)) {
        
        // Stop completely
        PORTA.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
        TCA0.SINGLE.CMP1 = 0;
        return;
    }

    uint8_t duty = 0;

    // FORWARD (joystick value LESS than idle)
    if (joystick_value < (THROTTLE_IDLE - THROTTLE_DEADBAND)) {
        PORTA.OUTCLR = (1 << MOTOR_IN1_PIN);  // IN1 = LOW
        PORTA.OUTSET = (1 << MOTOR_IN2_PIN);  // IN2 = HIGH

        uint8_t range = THROTTLE_IDLE - THROTTLE_DEADBAND;
        uint8_t scaled = range - joystick_value;

        // Stronger response, more aggressive scaling
        duty = MOTOR_MIN_POWER + ((scaled * (MOTOR_MAX_POWER - MOTOR_MIN_POWER)) / range);
    }

    // REVERSE (joystick value GREATER than idle)
    else {
        PORTA.OUTSET = (1 << MOTOR_IN1_PIN);  // IN1 = HIGH
        PORTA.OUTCLR = (1 << MOTOR_IN2_PIN);  // IN2 = LOW

        uint8_t range = 255 - (THROTTLE_IDLE + THROTTLE_DEADBAND);
        uint8_t scaled = joystick_value - (THROTTLE_IDLE + THROTTLE_DEADBAND);

        duty = MOTOR_MIN_POWER + ((scaled * (MOTOR_MAX_POWER - MOTOR_MIN_POWER)) / range);
    }

    if (duty > MOTOR_MAX_POWER) duty = MOTOR_MAX_POWER;

    // Apply duty
    TCA0.SINGLE.CMP1 = duty;
}


void control_headlight(bool on) {
    if (on) {
        PORTA.OUTSET = (1 << HEADLIGHT_PIN); // Headlight ON (active HIGH)
    } else {
        PORTA.OUTCLR = (1 << HEADLIGHT_PIN); // Headlight OFF
    }
}

//======== INPUT PROCESSING ========//

uint16_t read_adc(uint8_t pin) {
    // Select ADC input pin
    ADC0.MUXPOS = pin;
    
    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;
    
    // Wait for conversion to complete
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    
    // Clear flag
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    
    // Return result
    return ADC0.RES;
}

uint8_t get_filtered_value(uint16_t *buffer) {
    // Improved filtering using median approach from working code
    // Copy buffer to temp array
    uint16_t temp_buffer[FILTER_SIZE];
    
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        temp_buffer[i] = buffer[i];
    }
    
    // Simple bubble sort for small array (more robust than mean)
    for (uint8_t i = 0; i < FILTER_SIZE - 1; i++) {
        for (uint8_t j = 0; j < FILTER_SIZE - i - 1; j++) {
            if (temp_buffer[j] > temp_buffer[j + 1]) {
                uint16_t temp = temp_buffer[j];
                temp_buffer[j] = temp_buffer[j + 1];
                temp_buffer[j + 1] = temp;
            }
        }
    }
    
    // Get median value
    uint16_t median = temp_buffer[FILTER_SIZE / 2];
    
    // Scale from 12-bit (0-4095) to 8-bit (0-255)
    uint8_t scaled_value = (uint8_t)((median * 255UL) / 4095UL);
    
    return scaled_value;
}

bool is_signal_active(void) {
    // Check if we've received any changes in the signal recently
    // This is a simple timeout mechanism to detect controller disconnection
    
    if (current_time - last_activity_time > ACTIVITY_TIMEOUT) {
        return false;
    }
    return true;
}

void set_safe_state(void) {
    // Set all outputs to a safe state if connection is lost
    
    // Center steering
    TCA0.SINGLE.CMP0 = SERVO_CENTER;
    
    // Stop motor
    PORTA.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    TCA0.SINGLE.CMP1 = 0;
    current_duty = 0;
    target_duty = 0;
    
    // Turn off headlight
    PORTA.OUTCLR = (1 << HEADLIGHT_PIN);
}

//======== MAIN FUNCTION ========//

int main(void) {
    // System initialization
    init_clock();
    init_pins();
    init_adc();
    init_pwm_tca0();
    
    // Short delay for system stability
    _delay_ms(100);
    
    // Main control loop
    while (1) {
        current_time++;  // Simple counter for timeout detection
        
        // 1. Read analog inputs from ESP32
        uint16_t new_steering_raw = read_adc(STEERING_INPUT_PIN);
        uint16_t new_motor_raw = read_adc(MOTOR_INPUT_PIN);
        
        // Check for signal activity to detect disconnection
        if (abs((int32_t)new_steering_raw - (int32_t)steering_raw) > 10 ||
            abs((int32_t)new_motor_raw - (int32_t)motor_raw) > 10) {
            last_activity_time = current_time;
        }
        
        // Update raw values
        steering_raw = new_steering_raw;
        motor_raw = new_motor_raw;
        
        // Add to filter buffers
        steering_buffer[buffer_index] = steering_raw;
        motor_buffer[buffer_index] = motor_raw;
        buffer_index = (buffer_index + 1) % FILTER_SIZE;
        
        // 2. Get filtered values
        steering_value = get_filtered_value(steering_buffer);
        motor_value = get_filtered_value(motor_buffer);
        
        // 3. Read digital input for headlight
        headlight_on = (PORTD.IN & (1 << HEADLIGHT_INPUT_PIN));
        
        // 4. Update outputs based on signal status
        if (is_signal_active()) {
            // Normal operation - apply all control inputs
            control_steering(steering_value);
            control_motor(motor_value);
            control_headlight(headlight_on);
        } else {
            // No signal detected - set everything to safe state
            set_safe_state();
        }
        
        // 5. Main loop delay - approximately 10ms for ~100Hz update rate
        _delay_ms(10);
    }
    
    return 0;
}