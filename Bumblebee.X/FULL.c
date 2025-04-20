#define F_CPU 4000000UL  // Using 4MHz clock
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//======== PIN ASSIGNMENTS (FROM CAR PIN ASSIGNMENTS DOCUMENT) ========//

// Steering Servo
#define STEERING_SERVO_PIN 0         // PA0

// L298N Motor Driver
#define MOTOR_ENA_PIN 1              // PA1
#define MOTOR_IN1_PIN 2              // PA2
#define MOTOR_IN2_PIN 3              // PA3

// RGB LED
#define RGB_LED_R_PIN 6              // PA6
#define RGB_LED_G_PIN 7              // PA7
#define RGB_LED_B_PIN 4              // PA4

// White LEDs (Headlights)
#define WHITE_LED_PIN 5              // PA5

// Metal Detector Servo
#define METAL_DETECTOR_SERVO_PIN 0   // PC0

// HC-SR04 Distance Sensor
#define HC_SR04_TRIG_PIN 1           // PC1
#define HC_SR04_ECHO_PIN 2           // PC2

// TM1637 Display
#define TM1637_CLK_PIN 3             // PC3 - CLK
#define TM1637_DIO_PIN 1             // PD1 - DIO

// ESP32 Communication Pins
#define ESP32_STEERING_PIN 2         // PD2 - Steering Servo Data from ESP32
#define ESP32_MOTOR_PIN 3            // PD3 - Motor Driver Data from ESP32
#define ESP32_METAL_PIN 4            // PD4 - Metal Detector Servo Data from ESP32
#define ESP32_HEADLIGHT_PIN 5        // PD5 - Headlight Data from ESP32
#define ESP32_DISTANCE_PIN 6         // PD6 - Distance Data to ESP32
#define ESP32_RGB_PIN 7              // PD7 - RGB Data from ESP32

//======== CONSTANTS ========//

// TM1637 Display Settings
#define TM1637_BRIGHTNESS 0x0F      // Maximum brightness (0x08 to 0x0F)
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88 // Display ON with brightness

// Motor Control Constants
#define THROTTLE_IDLE 127            // Middle value for throttle (0-255)
#define THROTTLE_DEADBAND 15         // Deadband around idle
#define STEERING_MIDDLE 127          // Middle value for steering (0-255)
#define STEERING_DEADBAND 5          // Deadband around middle

// Servo Pulse Width Constants (for 50Hz PWM)
#define SERVO_MIN_PULSE 62           // ~1ms pulse (0 degrees)
#define SERVO_MID_PULSE 94           // ~1.5ms pulse (90 degrees)
#define SERVO_MAX_PULSE 125          // ~2ms pulse (180 degrees)

// Distance Sensor Constants
#define MAX_DISTANCE 400             // Maximum measurable distance in cm
#define DISTANCE_FILTER_SIZE 5       // Size of moving average filter

// 7-segment lookup for digits 0-9
const uint8_t digitToSegment[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

//======== GLOBAL VARIABLES ========//

// Variables for distance sensing
volatile uint16_t echo_start = 0;
volatile uint16_t echo_end = 0;
volatile uint8_t echo_done = 0;
uint16_t distance_buffer[DISTANCE_FILTER_SIZE] = {0};
uint8_t distance_buffer_index = 0;
uint8_t distance_buffer_filled = 0;
uint16_t current_distance = 0;

// Variables for motor and steering control
uint8_t motor_value = THROTTLE_IDLE;
uint8_t steering_value = STEERING_MIDDLE;
uint8_t metal_detector_state = 0;    // 0 = up, 1 = down
uint8_t headlight_state = 0;         // 0 = off, 1 = on
uint8_t rgb_state = 0;               // 0 = off, 1 = on
uint8_t rgb_color = 0;               // 0-7 color index

// Metal detector servo pulse control
volatile uint8_t metal_detector_pulse_active = 0;

// Other variables
uint8_t display_mode = 0;            // 0 = distance, 1 = motor/steering debug

//======== FUNCTION PROTOTYPES ========//

// Initialization Functions
void init_system(void);
void init_pins(void);
void init_timers(void);
void init_adc(void);
void init_interrupts(void);

// TM1637 Display Functions
void tm1637_init(void);
void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t b);
void tm1637_display(uint8_t segments[]);
void display_value(uint16_t value);
void display_distance(uint16_t distance);
void display_motor_debug(uint8_t motor_val, uint8_t steering_val);

// HC-SR04 Distance Sensor Functions
void trigger_distance_measurement(void);
uint16_t calculate_distance(void);
uint16_t apply_distance_filter(uint16_t new_distance);

// Motor Control Functions
void control_motor(uint8_t value);
void control_steering_servo(uint8_t value);
void control_metal_detector_servo(uint8_t state);
void set_headlights(uint8_t state);
void control_rgb_led(uint8_t state, uint8_t color);

// ESP32 Communication Functions
void read_esp32_inputs(void);
void send_distance_to_esp32(uint16_t distance);

// ADC Functions
uint16_t read_adc(uint8_t channel);

//======== INTERRUPT SERVICE ROUTINES ========//

// Timer overflow interrupt for distance measurement timeout
ISR(TCB0_INT_vect) {   
    // Reset everything for the next measurement
    TCB0.INTFLAGS = TCB_OVF_bm; // Clear the interrupt flag
    echo_done = 1;
}

// Pin change interrupt for echo pin
ISR(PORTC_PORT_vect) {
    // Check if it's the echo pin
    if (PORTC.INTFLAGS & (1 << HC_SR04_ECHO_PIN)) {
        // If echo pin is high, start the timer
        if (PORTC.IN & (1 << HC_SR04_ECHO_PIN)) {
            TCB0.CNT = 0; // Reset counter
            echo_start = 0;
            TCB0.CTRLA |= TCB_ENABLE_bm; // Start timer
        } 
        // If echo pin is low, stop the timer and calculate the pulse duration
        else {
            echo_end = TCB0.CNT;
            TCB0.CTRLA &= ~TCB_ENABLE_bm; // Stop timer
            echo_done = 1;
        }
        PORTC.INTFLAGS = (1 << HC_SR04_ECHO_PIN); // Clear the interrupt flag
    }
}

//======== INITIALIZATION FUNCTIONS ========//

void init_system(void) {
    // Initialize all components
    init_pins();
    init_timers();
    init_adc();
    init_interrupts();
    tm1637_init();
    
    sei(); // Enable global interrupts
}

void init_pins(void) {
    // Configure PORTA pins
    PORTA.DIRSET = (1 << STEERING_SERVO_PIN) |     // PA0: Steering Servo
                   (1 << MOTOR_ENA_PIN) |          // PA1: Motor ENA
                   (1 << MOTOR_IN1_PIN) |          // PA2: Motor IN1
                   (1 << MOTOR_IN2_PIN) |          // PA3: Motor IN2
                   (1 << RGB_LED_B_PIN) |          // PA4: RGB Blue
                   (1 << WHITE_LED_PIN) |          // PA5: White LEDs
                   (1 << RGB_LED_R_PIN) |          // PA6: RGB Red
                   (1 << RGB_LED_G_PIN);           // PA7: RGB Green
    
    // Configure PORTC pins
    PORTC.DIRSET = (1 << METAL_DETECTOR_SERVO_PIN) | // PC0: Metal Detector Servo
                   (1 << HC_SR04_TRIG_PIN) |        // PC1: HC-SR04 TRIG
                   (1 << TM1637_CLK_PIN);           // PC3: TM1637 CLK
    
    PORTC.DIRCLR = (1 << HC_SR04_ECHO_PIN);         // PC2: HC-SR04 ECHO (input)
    
    // Configure PORTD pins
    PORTD.DIRSET = (1 << TM1637_DIO_PIN) |         // PD1: TM1637 DIO
                   (1 << ESP32_DISTANCE_PIN);       // PD6: ESP32 Distance Output
    
    PORTD.DIRCLR = (1 << ESP32_STEERING_PIN) |     // PD2: ESP32 Steering Input
                   (1 << ESP32_MOTOR_PIN) |         // PD3: ESP32 Motor Input
                   (1 << ESP32_METAL_PIN) |         // PD4: ESP32 Metal Detector Input
                   (1 << ESP32_HEADLIGHT_PIN) |     // PD5: ESP32 Headlight Input
                   (1 << ESP32_RGB_PIN);            // PD7: ESP32 RGB Input
    
    // Initialize outputs to default states
    PORTA.OUTCLR = (1 << MOTOR_ENA_PIN) |          // Motor off
                   (1 << MOTOR_IN1_PIN) |          // Motor direction pins low
                   (1 << MOTOR_IN2_PIN) |          // Motor direction pins low
                   (1 << RGB_LED_R_PIN) |          // RGB LED off
                   (1 << RGB_LED_G_PIN) |          // RGB LED off
                   (1 << RGB_LED_B_PIN) |          // RGB LED off
                   (1 << WHITE_LED_PIN);           // White LEDs off
    
    PORTC.OUTCLR = (1 << HC_SR04_TRIG_PIN);        // Ensure TRIG is initially low
    
    // Initialize TM1637 pins high
    PORTC.OUTSET = (1 << TM1637_CLK_PIN);
    PORTD.OUTSET = (1 << TM1637_DIO_PIN);
}

void init_timers(void) {
    // Configure TCA0 for servo PWM control
    // - We'll use WO0 for steering servo (PA0)
    // - We'll use WO1 for motor control (ENA pin on PA1)
    
    // For 50Hz PWM (20ms period) at 4MHz with DIV64, we need period = 1250
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;     // Route TCA0 to PORTA
    
    TCA0.SINGLE.PER = 1250;  // 50Hz PWM (20ms at 62.5kHz tick rate)
    
    // Enable compare channels (WO0, WO1)
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | 
                        TCA_SINGLE_CMP0EN_bm |     // WO0 for steering servo
                        TCA_SINGLE_CMP1EN_bm;      // WO1 for motor control
    
    // Initialize servos to center/neutral positions
    TCA0.SINGLE.CMP0 = SERVO_MID_PULSE;            // Steering center position
    TCA0.SINGLE.CMP1 = 0;                          // Motor stopped
    
    // Start TCA0 with DIV64 prescaler (4MHz/64 = 62.5kHz)
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
    
    // Configure TCB0 for distance measurement timing
    // At 4MHz, with prescaler 2, each tick is 0.5us
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc;               // Select DIV2 prescaler
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;               // Select periodic interrupt mode
    TCB0.CCMP = 60000;                             // Set overflow value (30ms timeout)
    TCB0.INTCTRL = TCB_OVF_bm;                     // Enable overflow interrupt
    // Timer is enabled in the ISR when echo pin goes high
}

void init_adc(void) {
    SREG = 0b10000000;                             // Enable global interrupts
    VREF.ADC0REF = 0b10000101;                     // Set ADC reference to VDD
    ADC0.INTCTRL = 0b00000001;                     // Enable ADC interrupt flag polling
    ADC0.CTRLC = 0x00;                             // Minimum ADC clock division
    ADC0.CTRLA = 0b00000001;                       // 12-bit resolution, single conversion mode
}

void init_interrupts(void) {
    // Configure pin interrupt for HC-SR04 ECHO pin
    PORTC.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc; // Pull-up enabled, interrupt on both edges
}

//======== TM1637 DISPLAY FUNCTIONS ========//

void tm1637_init(void) {
    _delay_ms(50);
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_DATA);
    tm1637_stop();
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_DISPLAY_CTRL | (TM1637_BRIGHTNESS & 0x07));
    tm1637_stop();
    uint8_t clear_segments[4] = {0, 0, 0, 0};
    tm1637_display(clear_segments);
}

void tm1637_start(void) {
    PORTD.OUTSET = (1 << TM1637_DIO_PIN);
    PORTC.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTD.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    PORTC.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
}

void tm1637_stop(void) {
    PORTC.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTD.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    PORTC.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTD.OUTSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}

void tm1637_write_byte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
        PORTC.OUTCLR = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        if (b & 0x01)
            PORTD.OUTSET = (1 << TM1637_DIO_PIN);
        else
            PORTD.OUTCLR = (1 << TM1637_DIO_PIN);
        _delay_us(2);
        PORTC.OUTSET = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        b >>= 1;
    }
    PORTC.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTD.DIRCLR = (1 << TM1637_DIO_PIN);
    _delay_us(5);
    PORTC.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTC.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTD.DIRSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}

void tm1637_display(uint8_t segments[]) {
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_ADDR);
    for (uint8_t i = 0; i < 4; i++) {
        tm1637_write_byte(segments[i]);
    }
    tm1637_stop();
}

void display_value(uint16_t value) {
    uint8_t digits[4];
    digits[0] = (value / 1000) % 10;
    digits[1] = (value / 100) % 10;
    digits[2] = (value / 10) % 10;
    digits[3] = value % 10;
    uint8_t segments[4];
    for (uint8_t i = 0; i < 4; i++) {
        segments[i] = digitToSegment[digits[i]];
    }
    if (value < 1000) segments[0] = 0;
    if (value < 100) segments[1] = 0;
    if (value < 10) segments[2] = 0;
    tm1637_display(segments);
}

void display_distance(uint16_t distance) {
    // The same as display_value but ensures at least one digit is shown
    uint8_t digits[4];
    digits[0] = (distance / 1000) % 10;
    digits[1] = (distance / 100) % 10;
    digits[2] = (distance / 10) % 10;
    digits[3] = distance % 10;
    uint8_t segments[4];
    for (uint8_t i = 0; i < 4; i++) {
        segments[i] = digitToSegment[digits[i]];
    }
    if (distance < 1000) segments[0] = 0;
    if (distance < 100) segments[1] = 0;
    if (distance < 10) segments[2] = 0;
    
    // Always show at least one digit
    if (distance == 0) segments[3] = digitToSegment[0];
    
    tm1637_display(segments);
}

void display_motor_debug(uint8_t motor_val, uint8_t steering_val) {
    uint8_t m_tens = motor_val / 10;
    uint8_t m_ones = motor_val % 10;
    uint8_t s_tens = steering_val / 10;
    uint8_t s_ones = steering_val % 10;
    
    uint8_t segments[4];
    segments[0] = digitToSegment[m_tens];
    segments[1] = digitToSegment[m_ones];
    segments[2] = digitToSegment[s_tens];
    segments[3] = digitToSegment[s_ones];
    
    // Add decimal point to separate motor and steering values
    segments[1] |= 0x80;  // Add decimal point to second digit
    
    tm1637_display(segments);
}

//======== HC-SR04 DISTANCE SENSOR FUNCTIONS ========//

void trigger_distance_measurement(void) {
    // Make sure timer is stopped and interrupts are cleared
    TCB0.CTRLA &= ~TCB_ENABLE_bm;
    TCB0.INTFLAGS = TCB_OVF_bm;
    PORTC.INTFLAGS = (1 << HC_SR04_ECHO_PIN);
    
    // Reset echo variables
    echo_start = 0;
    echo_end = 0;
    echo_done = 0;
    
    // Send a 10us pulse to TRIG pin
    PORTC.OUTSET = (1 << HC_SR04_TRIG_PIN);
    _delay_us(10);
    PORTC.OUTCLR = (1 << HC_SR04_TRIG_PIN);
}

uint16_t calculate_distance(void) {
    // If no echo was received or timer overflowed
    if (echo_end == 0) {
        return 0;
    }
    
    uint16_t pulse_duration = echo_end;
    
    // Convert pulse duration to distance in cm
    // Sound travels at ~343m/s = 34300cm/s = 0.0343cm/us
    // Distance = (Time × Speed of Sound) ÷ 2
    // With 0.5us per timer tick: Distance = pulse_duration × 0.5 × 0.0343 ÷ 2
    // Simplified: Distance = pulse_duration × 0.008575 (approx)
    // To avoid floating point: Distance = pulse_duration ÷ 117 (approx)
    
    uint16_t distance = pulse_duration / 117;
    
    // Limit max distance to 400cm (HC-SR04 max range)
    if (distance > MAX_DISTANCE) {
        distance = 0; // Out of range
    }
    
    return distance;
}

uint16_t apply_distance_filter(uint16_t new_distance) {
    // Add new distance to buffer
    distance_buffer[distance_buffer_index] = new_distance;
    
    // Update buffer index
    distance_buffer_index = (distance_buffer_index + 1) % DISTANCE_FILTER_SIZE;
    
    // Update buffer filled status
    if (distance_buffer_index == 0) {
        distance_buffer_filled = 1;
    }
    
    // Calculate average
    uint32_t sum = 0;
    uint8_t count = distance_buffer_filled ? DISTANCE_FILTER_SIZE : distance_buffer_index;
    
    for (uint8_t i = 0; i < count; i++) {
        sum += distance_buffer[i];
    }
    
    // Return average (prevent division by zero)
    return (count > 0) ? (uint16_t)(sum / count) : 0;
}

//======== MOTOR & SERVO CONTROL FUNCTIONS ========//

void control_motor(uint8_t joystick_value) {
    // Force motor to stop in deadzone
    if (joystick_value >= (THROTTLE_IDLE - THROTTLE_DEADBAND) && 
        joystick_value <= (THROTTLE_IDLE + THROTTLE_DEADBAND)) {
        // COMPLETE MOTOR STOP - Force all pins LOW
        PORTA.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
        TCA0.SINGLE.CMP1 = 0;  // No PWM
        return;
    }
    
    uint8_t duty = 0;
    
    // Forward control (values LOWER than idle)
    if (joystick_value < (THROTTLE_IDLE - THROTTLE_DEADBAND)) {
        // Forward
        PORTA.OUTCLR = (1 << MOTOR_IN1_PIN);  // IN1 = LOW
        PORTA.OUTSET = (1 << MOTOR_IN2_PIN);  // IN2 = HIGH
        
        // Calculate duty cycle - map joystick value to motor speed
        // Map from deadzone edge to 0 ? 200 to 255 (higher minimum power)
        uint8_t range = THROTTLE_IDLE - THROTTLE_DEADBAND;
        duty = 200 + ((range - joystick_value) * 55) / range;
    }
    // Reverse control (values HIGHER than idle)
    else {
        // Reverse
        PORTA.OUTSET = (1 << MOTOR_IN1_PIN);  // IN1 = HIGH
        PORTA.OUTCLR = (1 << MOTOR_IN2_PIN);  // IN2 = LOW
        
        // Calculate duty cycle - map joystick value to motor speed
        // Map from deadzone edge to max ? 200 to 255 (higher minimum power)
        uint8_t range = 255 - (THROTTLE_IDLE + THROTTLE_DEADBAND);
        duty = 200 + ((joystick_value - (THROTTLE_IDLE + THROTTLE_DEADBAND)) * 55) / range;
    }
    
    // Ensure valid duty cycle range
    if (duty > 255) duty = 255;
    
    // Set motor speed
    TCA0.SINGLE.CMP1 = duty;
}

void control_steering_servo(uint8_t joystick_value) {
    static uint8_t prev_pulse_width = SERVO_MID_PULSE;  // Previous pulse width (for smoothing)
    
    // Center deadzone check
    if (joystick_value >= (STEERING_MIDDLE - STEERING_DEADBAND) && 
        joystick_value <= (STEERING_MIDDLE + STEERING_DEADBAND)) {
        // Center position
        prev_pulse_width = SERVO_MID_PULSE;
        TCA0.SINGLE.CMP0 = SERVO_MID_PULSE;
        return;
    }
    
    // Map joystick value to servo pulse width (0.8ms to 2.2ms)
    uint8_t target_pulse_width;
    
    if (joystick_value < STEERING_MIDDLE) {
        // Turn left - map from 0->middle to min->mid pulse
        target_pulse_width = SERVO_MIN_PULSE + 
                ((uint32_t)(joystick_value) * (SERVO_MID_PULSE - SERVO_MIN_PULSE)) / 
                (STEERING_MIDDLE - STEERING_DEADBAND);
    } else {
        // Turn right - map from middle->255 to mid->max pulse
        target_pulse_width = SERVO_MID_PULSE + 
                ((uint32_t)(joystick_value - (STEERING_MIDDLE + STEERING_DEADBAND)) * 
                (SERVO_MAX_PULSE - SERVO_MID_PULSE)) / 
                (255 - (STEERING_MIDDLE + STEERING_DEADBAND));
    }
    
    // Constrain pulse width to valid range
    if (target_pulse_width < SERVO_MIN_PULSE) target_pulse_width = SERVO_MIN_PULSE;
    if (target_pulse_width > SERVO_MAX_PULSE) target_pulse_width = SERVO_MAX_PULSE;
    
    // Smooth changes to reduce jitter (move only 1/4 of the way to target)
    uint8_t pulse_width = prev_pulse_width + ((target_pulse_width - prev_pulse_width) / 4);
    prev_pulse_width = pulse_width;
    
    // Set servo position
    TCA0.SINGLE.CMP0 = pulse_width;
}

void control_metal_detector_servo(uint8_t state) {
    // State 0 = UP, State 1 = DOWN
    
    // Since we had issues with TCB1 routing, we'll use direct pin control instead
    // This is a simplified approach that doesn't use the timer
    
    if (state == 0) {
        // For servo "up" position - generate a 1ms pulse
        PORTC.OUTSET = (1 << METAL_DETECTOR_SERVO_PIN);  // Set pin high
        _delay_us(1000);                                 // 1ms pulse
        PORTC.OUTCLR = (1 << METAL_DETECTOR_SERVO_PIN);  // Set pin low
    } else {
        // For servo "down" position - generate a 2ms pulse
        PORTC.OUTSET = (1 << METAL_DETECTOR_SERVO_PIN);  // Set pin high
        _delay_us(2000);                                 // 2ms pulse
        PORTC.OUTCLR = (1 << METAL_DETECTOR_SERVO_PIN);  // Set pin low
    }
    
    // Note: This approach blocks the CPU during the pulse
    // A better approach would be to use a timer for non-blocking operation
    // But for simplicity and to avoid the PORTMUX issue, we'll use this method
}

void set_headlights(uint8_t state) {
    if (state) {
        PORTA.OUTSET = (1 << WHITE_LED_PIN);  // Turn on headlights
    } else {
        PORTA.OUTCLR = (1 << WHITE_LED_PIN);  // Turn off headlights
    }
}

void control_rgb_led(uint8_t state, uint8_t color) {
    // Turn off RGB LED if state is 0
    if (state == 0) {
        PORTA.OUTCLR = (1 << RGB_LED_R_PIN) | (1 << RGB_LED_G_PIN) | (1 << RGB_LED_B_PIN);
        return;
    }
    
    // Basic 8 colors (RGB combinations)
    switch (color) {
        case 0: // Off
            PORTA.OUTCLR = (1 << RGB_LED_R_PIN) | (1 << RGB_LED_G_PIN) | (1 << RGB_LED_B_PIN);
            break;
        case 1: // Red
            PORTA.OUTSET = (1 << RGB_LED_R_PIN);
            PORTA.OUTCLR = (1 << RGB_LED_G_PIN) | (1 << RGB_LED_B_PIN);
            break;
        case 2: // Green
            PORTA.OUTSET = (1 << RGB_LED_G_PIN);
            PORTA.OUTCLR = (1 << RGB_LED_R_PIN) | (1 << RGB_LED_B_PIN);
            break;
        case 3: // Blue
            PORTA.OUTSET = (1 << RGB_LED_B_PIN);
            PORTA.OUTCLR = (1 << RGB_LED_R_PIN) | (1 << RGB_LED_G_PIN);
            break;
        case 4: // Yellow (Red + Green)
            PORTA.OUTSET = (1 << RGB_LED_R_PIN) | (1 << RGB_LED_G_PIN);
            PORTA.OUTCLR = (1 << RGB_LED_B_PIN);
            break;
        case 5: // Magenta (Red + Blue)
            PORTA.OUTSET = (1 << RGB_LED_R_PIN) | (1 << RGB_LED_B_PIN);
            PORTA.OUTCLR = (1 << RGB_LED_G_PIN);
            break;
        case 6: // Cyan (Green + Blue)
            PORTA.OUTSET = (1 << RGB_LED_G_PIN) | (1 << RGB_LED_B_PIN);
            PORTA.OUTCLR = (1 << RGB_LED_R_PIN);
            break;
        case 7: // White (Red + Green + Blue)
            PORTA.OUTSET = (1 << RGB_LED_R_PIN) | (1 << RGB_LED_G_PIN) | (1 << RGB_LED_B_PIN);
            break;
        default: // Default to off
            PORTA.OUTCLR = (1 << RGB_LED_R_PIN) | (1 << RGB_LED_G_PIN) | (1 << RGB_LED_B_PIN);
            break;
    }
}

//======== ESP32 COMMUNICATION FUNCTIONS ========//

void read_esp32_inputs(void) {
    // For simplicity, we'll read the ESP32 inputs as digital signals
    // In a more advanced implementation, we would use ADC for analog values
    
    // Read the metal detector state (digital)
    metal_detector_state = (PORTD.IN & (1 << ESP32_METAL_PIN)) ? 1 : 0;
    
    // Read the headlight state (digital)
    headlight_state = (PORTD.IN & (1 << ESP32_HEADLIGHT_PIN)) ? 1 : 0;
    
    // Read the RGB state (digital)
    // For simplicity, we just toggle RGB state and use a fixed color
    // In a real implementation, you would decode multiple pulses for color
    rgb_state = (PORTD.IN & (1 << ESP32_RGB_PIN)) ? 1 : 0;
    
    // Read steering value (digital for now - better with ADC)
    // This is a simplified implementation
    steering_value = (PORTD.IN & (1 << ESP32_STEERING_PIN)) ? 
                      STEERING_MIDDLE + 20 : STEERING_MIDDLE - 20;
    
    // Read motor value (digital for now - better with ADC)
    // This is a simplified implementation
    motor_value = (PORTD.IN & (1 << ESP32_MOTOR_PIN)) ? 
                   THROTTLE_IDLE + 20 : THROTTLE_IDLE - 20;
}

void send_distance_to_esp32(uint16_t distance) {
    // For a simple digital approach - just indicate if object is detected
    if (distance > 0 && distance < 100) {
        PORTD.OUTSET = (1 << ESP32_DISTANCE_PIN);  // Object detected
    } else {
        PORTD.OUTCLR = (1 << ESP32_DISTANCE_PIN);  // No object detected
    }
    
    // In a more advanced implementation, you would use pulse width modulation 
    // or a communication protocol to send the actual distance value
}

//======== ADC FUNCTIONS ========//

uint16_t read_adc(uint8_t channel) {
    // Set the ADC channel
    ADC0.MUXPOS = channel;
    
    // Start conversion
    ADC0.COMMAND = 0x01;
    
    // Wait for conversion to complete
    while (!(ADC0.INTFLAGS & 0x01)) { ; }
    
    // Read the result
    uint16_t value = ADC0.RES;
    
    // Clear the conversion complete flag
    ADC0.INTFLAGS = 0x01;
    
    return value;
}

//======== MAIN FUNCTION ========//

int main(void) {
    // Set clock to 4MHz
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;
    
    // Initialize all components
    init_system();
    
    // Display test pattern briefly
    uint8_t test_segments[4] = {0x7F, 0x7F, 0x7F, 0x7F}; // All segments on
    tm1637_display(test_segments);
    _delay_ms(500);
    
    // Display initial zero
    display_value(0);
    
    // Main control loop
    while (1) {
        // Read inputs from ESP32
        read_esp32_inputs();
        
        // Trigger distance measurement
        trigger_distance_measurement();
        
        // Wait for echo (with timeout)
        uint8_t timeout = 0;
        while (!echo_done && timeout < 100) {
            _delay_us(100);
            timeout++;
        }
        
        if (echo_done) {
            // Calculate and filter distance
            uint16_t new_distance = calculate_distance();
            current_distance = apply_distance_filter(new_distance);
            
            // Send distance to ESP32
            send_distance_to_esp32(current_distance);
            
            // Reset echo_done flag for next measurement
            echo_done = 0;
        }
        
        // Control motors and servos based on ESP32 input values
        control_motor(motor_value);
        control_steering_servo(steering_value);
        control_metal_detector_servo(metal_detector_state);
        set_headlights(headlight_state);
        control_rgb_led(rgb_state, rgb_color);
        
        // Display distance on 7-segment display
        display_distance(current_distance);
        
        // Delay between cycles
        _delay_ms(50);  // 20Hz update rate
    }
    
    return 0; // Never reached
}