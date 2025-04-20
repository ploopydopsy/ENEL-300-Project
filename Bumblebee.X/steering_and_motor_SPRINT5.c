#define F_CPU 4000000UL  // Using 4MHz as in the first code
//CODE HAS CONTROL OF BOTH STEERING SERVO AND MOTOR DRIVER 
//motor_steering_combo.c

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>  // Added for abs() function
#include <stdbool.h>

//======== PIN ASSIGNMENTS ========//

// TM1637 display pins
#define TM1637_CLK_PIN 7            // PD7
#define TM1637_DIO_PIN 6            // PD6
#define TM1637_PORT PORTD           // PORTD for TM1637

// Motor control pins
#define MOTOR_ENA_PIN 1             // PA1 (PWM output for speed using TCA0 WO1)
#define MOTOR_IN1_PIN 2             // PA2
#define MOTOR_IN2_PIN 3             // PA3
#define MOTOR_PORT PORTA            // PORTA for motor control

// Servo control pin
#define SERVO_PIN 0                 // PA0 (PWM output for steering using TCA0 WO0)

// Joystick pins
#define MOTOR_JOYSTICK_PIN 2        // PD2 (AIN2) - For forward/backward
#define STEERING_JOYSTICK_PIN 3     // PD3 (AIN3) - For left/right steering
#define JOYSTICK_PORT PORTD         // PORTD for joysticks

// Display settings
#define TM1637_BRIGHTNESS 0x0F      // Maximum brightness (0x08 to 0x0F)
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88 // Display ON with brightness

//CONSTANTS - Based on observed values from actual testing
#define THROTTLE_IDLE 80            // Updated idle value based on testing (78-82)
#define THROTTLE_DEADBAND 15        // Wide deadband around idle (65-95)
#define STEERING_MIDDLE 114         // Middle value for steering
#define STEERING_DEADBAND 5         // Deadband around middle

// Variables for display mode toggle
uint8_t display_mode = 0;           // 0 = Motor, 1 = Steering

// 7-segment lookup for digits 0-9
const uint8_t digitToSegment[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// Filter buffer variables
#define FILTER_SIZE 8
uint16_t throttle_buffer[FILTER_SIZE];
uint16_t steering_buffer[FILTER_SIZE];
uint8_t buffer_index = 0;

//======== FUNCTION PROTOTYPES ========//
void init_pins(void);
void init_pwm_and_servo(void);
void init_adc(void);

void tm1637_init(void);
void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t b);
void tm1637_display(uint8_t segments[]);
void display_value(uint16_t value);
void display_dual_values(uint8_t motor_val, uint8_t steering_val);
void display_motor_debug(uint8_t motor_val);

uint16_t read_adc(uint8_t channel);
uint8_t get_filtered_value(uint16_t *buffer);
void control_motor(uint8_t joystick_value);
void control_servo(uint8_t joystick_value);

//======== INITIALIZATION FUNCTIONS ========//

void init_pins(void) {
    // Configure TM1637 pins as outputs
    TM1637_PORT.DIRSET = (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    
    // Configure motor control pins as outputs
    MOTOR_PORT.DIRSET = (1 << MOTOR_ENA_PIN) | (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    
    // Configure servo pin as output
    PORTA.DIRSET = (1 << SERVO_PIN);
    
    // Configure joystick pins as inputs
    JOYSTICK_PORT.DIRCLR = (1 << MOTOR_JOYSTICK_PIN) | (1 << STEERING_JOYSTICK_PIN);
    
    // Initialize motor outputs LOW
    MOTOR_PORT.OUTCLR = (1 << MOTOR_ENA_PIN) | (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    
    // Initialize filter buffers
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        throttle_buffer[i] = 0;
        steering_buffer[i] = 0;
    }
}

void init_adc(void) {
    SREG = 0b10000000;              // Enable global interrupts
    VREF.ADC0REF = 0b10000101;      // Set ADC reference to VDD
    ADC0.INTCTRL = 0b00000001;      // Enable ADC interrupt flag polling
    ADC0.CTRLC = 0x00;              // Minimum ADC clock division
    ADC0.CTRLA = 0b00000001;        // 12-bit resolution, single conversion mode
}

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

// Calculate the median of the last FILTER_SIZE values
uint8_t get_filtered_value(uint16_t *buffer) {
    // Sort the buffer (simple bubble sort is OK for small arrays)
    uint16_t temp_buffer[FILTER_SIZE];
    
    // Copy buffer to temp buffer
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        temp_buffer[i] = buffer[i];
    }
    
    // Bubble sort temp buffer
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
    
    // Scale down to 0-255 range (8-bit)
    uint8_t scaled_value = (uint8_t)((median * 255UL) / 4095UL);
    
    return scaled_value;
}

void init_pwm_and_servo(void) {
    // Setup both motor and servo PWM on TCA0 (WO1 for motor, WO0 for servo)
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
    
    // Configure TCA0 for both motor control and servo
    // For motor: We use 8-bit PWM (0-255)
    // For servo: We need a 50Hz signal (20ms period)
    
    // At 4MHz with DIV64, tick frequency is 62.5kHz
    // For 50Hz, we need period = 62500/50 = 1250 ticks
    TCA0.SINGLE.PER = 1250;  
    
    // Enable both compare channels (WO0 for servo, WO1 for motor)
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | 
                      TCA_SINGLE_CMP0EN_bm | 
                      TCA_SINGLE_CMP1EN_bm;
    
    // Initialize motor to 0% duty
    TCA0.SINGLE.CMP1 = 0;
    
    // Initialize servo to center position (1.5ms = 94 ticks at 62.5kHz)
    TCA0.SINGLE.CMP0 = 94;  // ~1.5ms pulse (center position)
    
    // Start TCA0 with DIV64 prescaler
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
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

// Display motor debug info
void display_motor_debug(uint8_t motor_val) {
    uint8_t segments[4];
    
    // Check if in deadzone (motor should be stopped)
    if (motor_val >= (THROTTLE_IDLE - THROTTLE_DEADBAND) && 
        motor_val <= (THROTTLE_IDLE + THROTTLE_DEADBAND)) {
        // Display "St" for stop
        segments[0] = 0x6D; // S
        segments[1] = 0x78; // t
    } else if (motor_val < (THROTTLE_IDLE - THROTTLE_DEADBAND)) {
        // Display "Fd" for forward
        segments[0] = 0x71; // F
        segments[1] = 0x5E; // d
    } else {
        // Display "re" for reverse
        segments[0] = 0x50; // r
        segments[1] = 0x79; // e
    }
    
    // Show actual value on right side
    segments[2] = digitToSegment[motor_val / 10 % 10];
    segments[3] = digitToSegment[motor_val % 10];
    
    tm1637_display(segments);
}

// Display two values side by side (M:xxx S:xxx)
void display_dual_values(uint8_t motor_val, uint8_t steering_val) {
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

void tm1637_display(uint8_t segments[]) {
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_ADDR);
    for (uint8_t i = 0; i < 4; i++) {
        tm1637_write_byte(segments[i]);
    }
    tm1637_stop();
}

void tm1637_start(void) {
    TM1637_PORT.OUTSET = (1 << TM1637_DIO_PIN);
    TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    TM1637_PORT.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
}

void tm1637_stop(void) {
    TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    TM1637_PORT.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    TM1637_PORT.OUTSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}

void tm1637_write_byte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
        TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        if (b & 0x01)
            TM1637_PORT.OUTSET = (1 << TM1637_DIO_PIN);
        else
            TM1637_PORT.OUTCLR = (1 << TM1637_DIO_PIN);
        _delay_us(2);
        TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        b >>= 1;
    }
    TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
    TM1637_PORT.DIRCLR = (1 << TM1637_DIO_PIN);
    _delay_us(5);
    TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
    TM1637_PORT.DIRSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}

// ======== MOTOR CONTROL FUNCTION ======== //

void control_motor(uint8_t joystick_value) {
    // Force motor to stop in deadzone - EXTRA LARGE DEADBAND
    if (joystick_value >= (THROTTLE_IDLE - THROTTLE_DEADBAND) && 
        joystick_value <= (THROTTLE_IDLE + THROTTLE_DEADBAND)) {
        // COMPLETE MOTOR STOP - Force all pins LOW
        MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
        TCA0.SINGLE.CMP1 = 0;  // No PWM
        return;
    }
    
    uint8_t duty = 0;
    
    // Forward control (values LOWER than idle)
    if (joystick_value < (THROTTLE_IDLE - THROTTLE_DEADBAND)) {
        // Forward
        MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN);  // IN1 = LOW
        MOTOR_PORT.OUTSET = (1 << MOTOR_IN2_PIN);  // IN2 = HIGH
        
        // Calculate duty cycle - map joystick value to motor speed
        // Map from deadzone edge to 0 ? 200 to 255 (MUCH higher minimum power)
        uint8_t range = THROTTLE_IDLE - THROTTLE_DEADBAND;
        duty = 200 + ((range - joystick_value) * 55) / range;
    }
    // Reverse control (values HIGHER than idle)
    else {
        // Reverse
        MOTOR_PORT.OUTSET = (1 << MOTOR_IN1_PIN);  // IN1 = HIGH
        MOTOR_PORT.OUTCLR = (1 << MOTOR_IN2_PIN);  // IN2 = LOW
        
        // Calculate duty cycle - map joystick value to motor speed
        // Map from deadzone edge to max ? 200 to 255 (MUCH higher minimum power)
        uint8_t range = 255 - (THROTTLE_IDLE + THROTTLE_DEADBAND);
        duty = 200 + ((joystick_value - (THROTTLE_IDLE + THROTTLE_DEADBAND)) * 55) / range;
    }
    
    // Ensure valid duty cycle range
    if (duty > 255) duty = 255;
    
    // Set motor speed
    TCA0.SINGLE.CMP1 = duty;
}

// ======== SERVO CONTROL FUNCTION ======== //

void control_servo(uint8_t joystick_value) {
    static uint8_t prev_pulse_width = 94;  // Previous pulse width (for smoothing)
    
    // Map steering value to servo pulse width
    // From screenshot, middle value is around 114
    
    // Center deadzone check
    if (joystick_value >= STEERING_MIDDLE - STEERING_DEADBAND && 
        joystick_value <= STEERING_MIDDLE + STEERING_DEADBAND) {
        // Center position (1.5ms pulse = ~94 ticks at 62.5kHz)
        prev_pulse_width = 94;
        TCA0.SINGLE.CMP0 = 94;
        return;
    }
    
    // EVEN WIDER RANGE: Use 0.3ms to 2.7ms pulse width (19 to 169 ticks)
    uint8_t target_pulse_width;
    
    if (joystick_value < STEERING_MIDDLE) {
        // Turn left - map from 0->114 to 19->94
        target_pulse_width = 19 + ((uint32_t)(joystick_value) * 75) / (STEERING_MIDDLE - STEERING_DEADBAND);
    } else {
        // Turn right - map from 114->255 to 94->169 
        target_pulse_width = 94 + ((uint32_t)(joystick_value - (STEERING_MIDDLE + STEERING_DEADBAND)) * 75) / 
                     (255 - (STEERING_MIDDLE + STEERING_DEADBAND));
    }
    
    // Constrain pulse width to valid range
    if (target_pulse_width < 19) target_pulse_width = 19;     // 0.3ms minimum
    if (target_pulse_width > 169) target_pulse_width = 169;   // 2.7ms maximum
    
    // Smooth changes to reduce jitter (move only 1/4 of the way to target)
    uint8_t pulse_width = prev_pulse_width + ((target_pulse_width - prev_pulse_width) / 4);
    prev_pulse_width = pulse_width;
    
    // Set servo position
    TCA0.SINGLE.CMP0 = pulse_width;
}

// ======== MAIN FUNCTION ======== //

int main(void) {
    // Set clock to 4MHz to match F_CPU
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;
    
    // Initialize components
    init_pins();
    tm1637_init();
    init_adc();
    init_pwm_and_servo();
    
    sei();  // Enable interrupts
    
    // Display test pattern to verify display works
    uint8_t test_segments[4] = {0x7F, 0x7F, 0x7F, 0x7F}; // All segments on
    tm1637_display(test_segments);
    _delay_ms(1000);
    
    // Display initial zero
    display_value(0);
    _delay_ms(500);
    
    // Stop motor initially
    MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    TCA0.SINGLE.CMP1 = 0;
    
    while (1) {
        // Read raw ADC values
        uint16_t throttle_raw = read_adc(MOTOR_JOYSTICK_PIN);
        uint16_t steering_raw = read_adc(STEERING_JOYSTICK_PIN);
        
        // Add values to filter buffers
        throttle_buffer[buffer_index] = throttle_raw;
        steering_buffer[buffer_index] = steering_raw;
        buffer_index = (buffer_index + 1) % FILTER_SIZE;
        
        // Get filtered values
        uint8_t throttle_filtered = get_filtered_value(throttle_buffer);
        uint8_t steering_filtered = get_filtered_value(steering_buffer);
        
        // Display motor debug info (status and value)
        display_motor_debug(throttle_filtered);
        
        // MOTOR DEADBAND CHECK - Force motor to stop in deadzone
        if (throttle_filtered >= (THROTTLE_IDLE - THROTTLE_DEADBAND) && 
            throttle_filtered <= (THROTTLE_IDLE + THROTTLE_DEADBAND)) {
            // Within deadzone - force motor to stop completely
            MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
            TCA0.SINGLE.CMP1 = 0;  // No PWM
        }
        else {
            // Outside deadzone - control motor normally
            control_motor(throttle_filtered);
        }
        
        // Control servo - THIS PART IS WORKING, DON'T MODIFY
        control_servo(steering_filtered);
        
        _delay_ms(10);  // Update at 100Hz
    }
}