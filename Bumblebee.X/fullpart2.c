#define F_CPU 4000000UL  // 4MHz clock
// failed main from when i was tryna get it all to work on one avr. it has working servo and motor driver functionality. it has working headlights. RGB does not work. the stuff thats now on avr 2 didnt work on heree but the faulty stuff is still here
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

//======== PIN DEFINITIONS ========//

// Steering Servo and Motor Control (TCA0)
#define SERVO_PIN 0           // PA0 (TCA0 WO0) - Steering servo
#define MOTOR_ENA_PIN 1       // PA1 (TCA0 WO1) - Motor speed control (ENA)
#define MOTOR_IN1_PIN 2       // PA2 - Motor direction control
#define MOTOR_IN2_PIN 3       // PA3 - Motor direction control

// RGB LED and Headlight
#define RGB_BLUE_PIN 4        // PA4 - RGB Blue (common anode)
#define HEADLIGHT_PIN 5       // PA5 - White LED headlights
#define RGB_RED_PIN 6         // PA6 - RGB Red (common anode)
#define RGB_GREEN_PIN 7       // PA7 - RGB Green (common anode)

// Metal Detector Servo
#define METAL_SERVO_PIN 0     // PC0 (TCB2 WO) - Metal detector servo

// Distance Sensor
#define ECHO_PIN 1            // PC1 (TCB1) - HC-SR04 Echo input
#define TRIG_PIN 2            // PC2 - HC-SR04 Trigger output

// TM1637 Display
#define TM1637_CLK_PIN 3      // PC3 - TM1637 display CLK
#define TM1637_DIO_PIN 1      // PD1 - TM1637 display DIO

// ESP32 Communication Pins
#define STEERING_INPUT_PIN 2  // PD2 - Steering control from ESP32
#define MOTOR_INPUT_PIN 3     // PD3 - Motor control from ESP32
#define METAL_INPUT_PIN 4     // PD4 - Metal detector toggle from ESP32
#define HEADLIGHT_INPUT_PIN 5 // PD5 - Headlight toggle from ESP32
#define DISTANCE_OUTPUT_PIN 6 // PD6 - Distance value to ESP32
#define RGB_INPUT_PIN 7       // PD7 - RGB control from ESP32

//======== CONSTANTS & SETTINGS ========//

// Servo control constants
#define SERVO_CENTER 94       // 1.5ms pulse (center position)
#define SERVO_MIN 19          // 0.3ms pulse (left position)
#define SERVO_MAX 169         // 2.7ms pulse (right position)
#define SERVO_DEADBAND 5      // Deadband around center position

// Motor control constants
#define MOTOR_IDLE 80         // Motor stop value
#define MOTOR_DEADBAND 15     // Deadband around idle

// Metal detector servo positions
#define METAL_SERVO_UP 1000   // ~1ms pulse (up position)
#define METAL_SERVO_DOWN 2000 // ~2ms pulse (down position)

// TM1637 Display settings
#define TM1637_BRIGHTNESS 0x0F // Maximum brightness (0x08 to 0x0F)
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88 // Display ON with brightness

// RGB color lookup table (common anode: LOW = ON, HIGH = OFF)
const bool RGB_COLORS[8][3] = {
    {true, true, true},    // 0 = OFF (all pins HIGH)
    {false, true, true},   // 1 = RED (only R pin LOW)
    {true, false, true},   // 2 = GREEN (only G pin LOW)
    {true, true, false},   // 3 = BLUE (only B pin LOW)
    {false, false, true},  // 4 = YELLOW (R+G pins LOW)
    {false, true, false},  // 5 = MAGENTA (R+B pins LOW)
    {true, false, false},  // 6 = CYAN (G+B pins LOW)
    {false, false, false}  // 7 = WHITE (all pins LOW)
};

// 7-segment lookup for digits 0-9
const uint8_t digitToSegment[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

//======== VARIABLES ========//

// Input filter buffers
#define FILTER_SIZE 8
uint16_t throttle_buffer[FILTER_SIZE];
uint16_t steering_buffer[FILTER_SIZE];
uint8_t buffer_index = 0;

// Distance measurement variables
volatile uint16_t echo_start = 0;
volatile uint16_t echo_end = 0;
volatile uint8_t echo_done = 0;
uint16_t distance_cm = 0;

// Distance filter
#define DIST_FILTER_SIZE 5
uint16_t distance_buffer[DIST_FILTER_SIZE] = {0};
uint8_t dist_buffer_index = 0;
uint8_t dist_buffer_filled = 0;

// RGB LED state
uint8_t rgb_color_index = 0;
bool rgb_led_on = false;

//======== FUNCTION PROTOTYPES ========//

// Initialization functions
void init_clock(void);
void init_pins(void);
void init_adc(void);
void init_pwm_tca0(void);  // For steering servo and motor
void init_pwm_tcb2(void);  // For metal detector servo
void init_timer_tcb1(void); // For distance sensor (changed from TCB3 to TCB1)

// TM1637 Display functions
void tm1637_init(void);
void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t b);
void tm1637_display(uint8_t segments[]);
void display_distance(uint16_t distance);

// Control functions
void control_motor(uint8_t joystick_value);
void control_steering(uint8_t joystick_value);
void control_metal_detector(bool down_position);
void control_rgb_led(uint8_t color_index, bool on);
void control_headlight(bool on);

// Sensor functions
void trigger_distance_measurement(void);
uint16_t calculate_distance(void);
uint16_t apply_distance_filter(uint16_t new_distance);

// Input processing
uint16_t read_adc(uint8_t channel);
uint8_t get_filtered_value(uint16_t *buffer);

//======== INTERRUPT HANDLERS ========//

// Timer overflow interrupt for distance sensor timeout
ISR(TCB1_INT_vect) {  // Changed from TCB3_INT_vect to TCB1_INT_vect
    // This interrupt will trigger if the echo is too long
    TCB1.INTFLAGS = TCB_OVF_bm; // Clear the interrupt flag
    echo_done = 1; // Signal measurement is done (timeout)
}

// Pin change interrupt for ECHO pin
ISR(PORTC_PORT_vect) {
    // Check if it's the echo pin
    if (PORTC.INTFLAGS & (1 << ECHO_PIN)) {
        // If echo pin is high, start the timer
        if (PORTC.IN & (1 << ECHO_PIN)) {
            TCB1.CNT = 0; // Reset counter
            echo_start = 0;
            TCB1.CTRLA |= TCB_ENABLE_bm; // Start timer
        } 
        // If echo pin is low, stop the timer and calculate the pulse duration
        else {
            echo_end = TCB1.CNT;
            TCB1.CTRLA &= ~TCB_ENABLE_bm; // Stop timer
            echo_done = 1; // Signal measurement is done
        }
        PORTC.INTFLAGS = (1 << ECHO_PIN); // Clear the interrupt flag
    }
}

//======== INITIALIZATION FUNCTIONS ========//

void init_clock(void) {
    // Set up 4MHz internal oscillator
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;
}

void init_pins(void) {
    // PORTA - Servo, Motor Control, RGB LED, Headlight
    PORTA.DIRSET = (1 << SERVO_PIN) | (1 << MOTOR_ENA_PIN) | 
                  (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN) |
                  (1 << RGB_RED_PIN) | (1 << RGB_GREEN_PIN) | 
                  (1 << RGB_BLUE_PIN) | (1 << HEADLIGHT_PIN);
    
    // Set RGB LEDs OFF initially (common anode, HIGH = OFF)
    PORTA.OUTSET = (1 << RGB_RED_PIN) | (1 << RGB_GREEN_PIN) | (1 << RGB_BLUE_PIN);
    
    // Set headlight OFF initially
    PORTA.OUTCLR = (1 << HEADLIGHT_PIN);
    
    // Initialize motor direction pins LOW
    PORTA.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    
    // PORTC - Metal detector servo, HC-SR04, TM1637 CLK
    PORTC.DIRSET = (1 << METAL_SERVO_PIN) | (1 << TRIG_PIN) | (1 << TM1637_CLK_PIN);
    PORTC.DIRCLR = (1 << ECHO_PIN);  // Echo pin as input
    
    // Initialize HC-SR04 pins
    PORTC.OUTCLR = (1 << TRIG_PIN);  // Trigger starts LOW
    
    // Configure echo pin for interrupts
    PORTC.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc; // Pull-up, interrupt on both edges
    
    // Set TM1637 CLK high initially
    PORTC.OUTSET = (1 << TM1637_CLK_PIN);
    
    // PORTD - TM1637 DIO, ESP32 communication pins
    PORTD.DIRSET = (1 << TM1637_DIO_PIN) | (1 << DISTANCE_OUTPUT_PIN);
    PORTD.DIRCLR = (1 << STEERING_INPUT_PIN) | (1 << MOTOR_INPUT_PIN) | 
                  (1 << METAL_INPUT_PIN) | (1 << HEADLIGHT_INPUT_PIN) |
                  (1 << RGB_INPUT_PIN);
    
    // Set TM1637 DIO high initially
    PORTD.OUTSET = (1 << TM1637_DIO_PIN);
    
    // Initialize filter buffers
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        throttle_buffer[i] = 0;
        steering_buffer[i] = 0;
    }
}

void init_adc(void) {
    SREG = 0b10000000;         // Enable global interrupts
    VREF.ADC0REF = 0b10000101; // Set ADC reference to VDD
    ADC0.INTCTRL = 0b00000001; // Enable ADC interrupt flag polling
    ADC0.CTRLC = 0x00;         // Minimum ADC clock division
    ADC0.CTRLA = 0b00000001;   // 12-bit resolution, single conversion mode
}

void init_pwm_tca0(void) {
    // Configure TCA0 for both servo and motor PWM
    
    // Route TCA0 to PORTA
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
    
    // At 4MHz with DIV64, tick frequency is 62.5kHz
    // For 50Hz servo signal, we need 1250 ticks per period (20ms)
    TCA0.SINGLE.PER = 1250;
    
    // Enable both compare channels (WO0 for servo, WO1 for motor)
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

void init_pwm_tcb2(void) {
    // Configure TCB2 for metal detector servo PWM on PC0
    
    // Use TCB2 in PWM 8-bit mode with PC0 as output
    TCB2.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm;
    
    // At 4MHz with DIV2, tick frequency is 2MHz
    // For 50Hz PWM (20ms period), we need CCMP = 40000
    TCB2.CCMP = 40000;  // Combined 8-bit period (MSB) and duty cycle (LSB)
    
    // Set initial position to UP (1ms pulse)
    // To avoid overflow, calculate in steps:
    uint16_t duty_scaled = ((uint32_t)METAL_SERVO_UP * 255) / 20000;  // Convert to 8-bit duty cycle
    TCB2.CCMPH = 200;    // Period high byte
    TCB2.CCMPL = duty_scaled;   // Duty cycle
    
    // Start TCB2 with DIV2 prescaler
    TCB2.CTRLA = TCB_CLKSEL_DIV2_gc | TCB_ENABLE_bm;
}

void init_timer_tcb1(void) {
    // Configure TCB1 for distance sensor echo timing (changed from TCB3 to TCB1)
    
    // Use TCB1 in periodic interrupt mode
    TCB1.CTRLB = TCB_CNTMODE_INT_gc;
    
    // At 4MHz with DIV2, tick frequency is 2MHz (0.5us per tick)
    // Set max timeout to 30ms (60000 ticks)
    TCB1.CCMP = 60000;
    
    // Enable overflow interrupt
    TCB1.INTCTRL = TCB_OVF_bm;
    
    // Don't enable the timer yet - we'll enable it when needed
    TCB1.CTRLA = TCB_CLKSEL_DIV2_gc;  // Configure clock but don't enable
}

//======== TM1637 DISPLAY FUNCTIONS ========//

void tm1637_init(void) {
    _delay_ms(50);  // Allow display to stabilize
    
    // Set data command
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_DATA);
    tm1637_stop();
    
    // Set display control (brightness)
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_DISPLAY_CTRL | (TM1637_BRIGHTNESS & 0x07));
    tm1637_stop();
    
    // Clear display initially
    uint8_t clear[4] = {0, 0, 0, 0};
    tm1637_display(clear);
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
    
    // Wait for ACK
    PORTC.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTD.DIRCLR = (1 << TM1637_DIO_PIN);  // Set DIO as input for ACK
    _delay_us(5);
    PORTC.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTC.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTD.DIRSET = (1 << TM1637_DIO_PIN);  // Set DIO back to output
    _delay_us(2);
}

void tm1637_display(uint8_t segments[]) {
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_ADDR);  // Set starting address to 0
    
    for (uint8_t i = 0; i < 4; i++) {
        tm1637_write_byte(segments[i]);
    }
    
    tm1637_stop();
}

void display_distance(uint16_t distance) {
    uint8_t digits[4];
    digits[0] = (distance / 1000) % 10;
    digits[1] = (distance / 100) % 10;
    digits[2] = (distance / 10) % 10;
    digits[3] = distance % 10;
    
    uint8_t segments[4];
    for (uint8_t i = 0; i < 4; i++) {
        segments[i] = digitToSegment[digits[i]];
    }
    
    // Don't show leading zeros
    if (distance < 1000) segments[0] = 0;
    if (distance < 100) segments[1] = 0;
    if (distance < 10) segments[2] = 0;
    
    // Special case: if all digits would be blank, show at least a zero
    if (distance == 0) segments[3] = digitToSegment[0];
    
    tm1637_display(segments);
}

//======== CONTROL FUNCTIONS ========//

void control_motor(uint8_t joystick_value) {
    // Check if in deadzone (motor should be stopped)
    if (joystick_value >= (MOTOR_IDLE - MOTOR_DEADBAND) && 
        joystick_value <= (MOTOR_IDLE + MOTOR_DEADBAND)) {
        // COMPLETE MOTOR STOP - Force all pins LOW
        PORTA.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
        TCA0.SINGLE.CMP1 = 0;  // No PWM
        return;
    }
    
    uint8_t duty = 0;
    
    // Forward control (values LOWER than idle)
    if (joystick_value < (MOTOR_IDLE - MOTOR_DEADBAND)) {
        // Forward
        PORTA.OUTCLR = (1 << MOTOR_IN1_PIN);  // IN1 = LOW
        PORTA.OUTSET = (1 << MOTOR_IN2_PIN);  // IN2 = HIGH
        
        // Map from deadzone edge to 0 -> 200 to 255 (higher minimum power)
        uint8_t range = MOTOR_IDLE - MOTOR_DEADBAND;
        duty = 200 + ((range - joystick_value) * 55) / range;
    }
    // Reverse control (values HIGHER than idle)
    else {
        // Reverse
        PORTA.OUTSET = (1 << MOTOR_IN1_PIN);  // IN1 = HIGH
        PORTA.OUTCLR = (1 << MOTOR_IN2_PIN);  // IN2 = LOW
        
        // Map from deadzone edge to max -> 200 to 255 (higher minimum power)
        uint8_t range = 255 - (MOTOR_IDLE + MOTOR_DEADBAND);
        duty = 200 + ((joystick_value - (MOTOR_IDLE + MOTOR_DEADBAND)) * 55) / range;
    }
    
    // Constrain duty cycle
    if (duty > 255) duty = 255;
    
    // Set motor speed
    TCA0.SINGLE.CMP1 = duty;
}

void control_steering(uint8_t joystick_value) {
    static uint8_t prev_pulse_width = SERVO_CENTER;  // For smoothing
    uint8_t target_pulse_width;
    
    // Center deadzone check
    if (joystick_value >= (128 - SERVO_DEADBAND) && 
        joystick_value <= (128 + SERVO_DEADBAND)) {
        target_pulse_width = SERVO_CENTER;
    }
    // Map joystick to servo range
    else if (joystick_value < 128) {
        // Turn left - map from 0->128 to MIN->CENTER
        target_pulse_width = SERVO_MIN + ((uint32_t)(joystick_value) * (SERVO_CENTER - SERVO_MIN)) / 128;
    } 
    else {
        // Turn right - map from 128->255 to CENTER->MAX 
        target_pulse_width = SERVO_CENTER + ((uint32_t)(joystick_value - 128) * (SERVO_MAX - SERVO_CENTER)) / 127;
    }
    
    // Constrain pulse width
    if (target_pulse_width < SERVO_MIN) target_pulse_width = SERVO_MIN;
    if (target_pulse_width > SERVO_MAX) target_pulse_width = SERVO_MAX;
    
    // Smooth changes to reduce jitter (move only 1/4 of the way to target)
    uint8_t pulse_width = prev_pulse_width + ((target_pulse_width - prev_pulse_width) / 4);
    prev_pulse_width = pulse_width;
    
    // Set servo position
    TCA0.SINGLE.CMP0 = pulse_width;
}

void control_metal_detector(bool down_position) {
    // Calculate 8-bit duty value for TCB2 PWM
    // To avoid overflow, calculate in steps:
    uint16_t duty_scaled;
    
    if (down_position) {
        // Metal detector down
        duty_scaled = ((uint32_t)METAL_SERVO_DOWN * 255) / 20000;
    } else {
        // Metal detector up
        duty_scaled = ((uint32_t)METAL_SERVO_UP * 255) / 20000;
    }
    
    // Update TCB2 duty cycle
    TCB2.CCMPL = duty_scaled;
}

void control_rgb_led(uint8_t color_index, bool on) {
    if (!on || color_index >= 8) {
        // LED is off or invalid color - turn all OFF
        PORTA.OUTSET = (1 << RGB_RED_PIN) | (1 << RGB_GREEN_PIN) | (1 << RGB_BLUE_PIN);
        return;
    }
    
    // Set each color channel based on lookup table (common anode: LOW = ON, HIGH = OFF)
    if (RGB_COLORS[color_index][0]) {
        PORTA.OUTSET = (1 << RGB_RED_PIN);   // RED OFF
    } else {
        PORTA.OUTCLR = (1 << RGB_RED_PIN);   // RED ON
    }
    
    if (RGB_COLORS[color_index][1]) {
        PORTA.OUTSET = (1 << RGB_GREEN_PIN); // GREEN OFF
    } else {
        PORTA.OUTCLR = (1 << RGB_GREEN_PIN); // GREEN ON
    }
    
    if (RGB_COLORS[color_index][2]) {
        PORTA.OUTSET = (1 << RGB_BLUE_PIN);  // BLUE OFF
    } else {
        PORTA.OUTCLR = (1 << RGB_BLUE_PIN);  // BLUE ON
    }
}

void control_headlight(bool on) {
    if (on) {
        PORTA.OUTSET = (1 << HEADLIGHT_PIN); // Headlight ON
    } else {
        PORTA.OUTCLR = (1 << HEADLIGHT_PIN); // Headlight OFF
    }
}

//======== DISTANCE SENSOR FUNCTIONS ========//

void trigger_distance_measurement(void) {
    // Make sure timer is stopped and interrupts are cleared
    TCB1.CTRLA &= ~TCB_ENABLE_bm;
    TCB1.INTFLAGS = TCB_OVF_bm;
    PORTC.INTFLAGS = (1 << ECHO_PIN);
    
    // Reset variables
    echo_done = 0;
    echo_end = 0;
    
    // Send 10us trigger pulse
    PORTC.OUTSET = (1 << TRIG_PIN);
    _delay_us(10);
    PORTC.OUTCLR = (1 << TRIG_PIN);
}

uint16_t calculate_distance(void) {
    // If no echo was received or timer overflowed
    if (echo_end == 0) {
        return 0;
    }
    
    // At 2MHz timer (0.5us per tick)
    // Distance = pulse_duration × 0.5us × 0.0343cm/us ÷ 2
    // Simplified: Distance = pulse_duration ÷ 117
    uint16_t distance = echo_end / 117;
    
    // Limit max distance to 400cm (HC-SR04 max range)
    if (distance > 400) {
        distance = 0;  // Out of range
    }
    
    return distance;
}

uint16_t apply_distance_filter(uint16_t new_distance) {
    // Add new distance to buffer
    distance_buffer[dist_buffer_index] = new_distance;
    
    // Update buffer index
    dist_buffer_index = (dist_buffer_index + 1) % DIST_FILTER_SIZE;
    
    // Mark buffer as filled once we've gone through it once
    if (dist_buffer_index == 0) {
        dist_buffer_filled = 1;
    }
    
    // Calculate average
    uint32_t sum = 0;
    uint8_t count = dist_buffer_filled ? DIST_FILTER_SIZE : dist_buffer_index;
    
    for (uint8_t i = 0; i < count; i++) {
        sum += distance_buffer[i];
    }
    
    // Return average (prevent division by zero)
    return (count > 0) ? (uint16_t)(sum / count) : 0;
}

//======== INPUT PROCESSING ========//

uint16_t read_adc(uint8_t channel) {
    // Set ADC channel
    ADC0.MUXPOS = channel;
    
    // Start conversion
    ADC0.COMMAND = 0x01;
    
    // Wait for conversion to complete
    while (!(ADC0.INTFLAGS & 0x01)) { ; }
    
    // Read result
    uint16_t value = ADC0.RES;
    
    // Clear flag
    ADC0.INTFLAGS = 0x01;
    
    return value;
}

uint8_t get_filtered_value(uint16_t *buffer) {
    // Simple mean filter for now (can be upgraded to median if needed)
    uint32_t sum = 0;
    
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        sum += buffer[i];
    }
    
    uint16_t mean = sum / FILTER_SIZE;
    
    // Scale down to 0-255 range (8-bit)
    uint8_t scaled_value = (uint8_t)((mean * 255UL) / 4095UL);
    
    return scaled_value;
}

//======== MAIN FUNCTION ========//

int main(void) {
    // System initialization
    init_clock();
    init_pins();
    init_adc();
    init_pwm_tca0();
    init_pwm_tcb2();
    init_timer_tcb1();  // Changed from init_timer_tcb3 to init_timer_tcb1
    tm1637_init();
    
    // Enable global interrupts
    sei();
    
    // Display initialization pattern (all segments on, then clear)
    uint8_t test_pattern[4] = {0x7F, 0x7F, 0x7F, 0x7F};
    tm1637_display(test_pattern);
    _delay_ms(500);
    display_distance(0);
    
    // Main loop
    uint32_t last_distance_time = 0;
    uint32_t current_time = 0;
    
    while (1) {
        //== 1. Distance Measurement (every 100ms) ==//
        current_time++;  // Simple counter as a time reference
        
        if (current_time - last_distance_time >= 10) {  // Approximately 100ms with the loop delay
            last_distance_time = current_time;
            
            // Trigger new measurement
            trigger_distance_measurement();
            
            // Wait for measurement to complete or timeout
            uint8_t timeout_counter = 0;
            while (!echo_done && timeout_counter < 100) {
                _delay_us(100);
                timeout_counter++;
            }
            
            // Calculate and filter distance
            uint16_t raw_distance = calculate_distance();
            distance_cm = apply_distance_filter(raw_distance);
            
            // Update display with current distance
            display_distance(distance_cm);
            
            // Send distance to ESP32 for wireless transmission
            // For simplicity, we're using a digital pin to indicate presence detection
            // In a more advanced implementation, you could use a protocol like UART
            if (distance_cm < 100) {
                PORTD.OUTSET = (1 << DISTANCE_OUTPUT_PIN);  // Object detected nearby
            } else {
                PORTD.OUTCLR = (1 << DISTANCE_OUTPUT_PIN);  // No object detected nearby
            }
        }
        
        //== 2. Read Input from ESP32 ==//
        
        // Read ESP32 communication pins
        bool metal_detector_down = (PORTD.IN & (1 << METAL_INPUT_PIN));
        bool headlight_on = (PORTD.IN & (1 << HEADLIGHT_INPUT_PIN));
        
        // Extract RGB control information
        bool rgb_on = (PORTD.IN & (1 << RGB_INPUT_PIN)) > 0; // RGB power state from ESP32
        
        // Read analog values for motor and steering
        uint16_t motor_raw = read_adc(MOTOR_INPUT_PIN);
        uint16_t steering_raw = read_adc(STEERING_INPUT_PIN);
        
        // Add to filter buffers
        throttle_buffer[buffer_index] = motor_raw;
        steering_buffer[buffer_index] = steering_raw;
        buffer_index = (buffer_index + 1) % FILTER_SIZE;
        
        // Get filtered values
        uint8_t motor_value = get_filtered_value(throttle_buffer);
        uint8_t steering_value = get_filtered_value(steering_buffer);
        
        // Get RGB color (analog value)
        uint16_t rgb_color_raw = read_adc(RGB_INPUT_PIN);
        rgb_color_index = (rgb_color_raw * 8) / 4096;  // Map 0-4095 to 0-7
        
        //== 3. Control Actuators ==//
        
        // Update motor and steering
        control_motor(motor_value);
        control_steering(steering_value);
        
        // Update metal detector servo
        control_metal_detector(metal_detector_down);
        
        // Update RGB LED
        control_rgb_led(rgb_color_index, rgb_on);
        
        // Update headlight
        control_headlight(headlight_on);
        
        // Small delay to prevent excessive CPU usage
        _delay_ms(10);
    }
    
    return 0;
}