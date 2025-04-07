#define F_CPU 4000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

// TM1637 display pins
#define TM1637_CLK_PIN 7  // PD7
#define TM1637_DIO_PIN 6  // PD6
#define TM1637_PORT PORTD

// Output pins
#define LED_PIN 5        // PD5
#define MOTOR_PIN 1      // PA1
#define SERVO_PIN 2      // PA2

// Input pins from RC receiver
#define RC_THROTTLE_PIN 2  // PD2
#define RC_STEERING_PIN 3  // PD3
#define RC_AUX_PIN 4       // PD4

// Display settings
#define TM1637_BRIGHTNESS 0x0F
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88

// Status codes for display
#define DISPLAY_CONNECTED 1111
#define DISPLAY_NOT_CONNECTED 0000

// 7-segment lookup for digits 0-9
const uint8_t digitToSegment[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// Variables to store RC pulse widths
volatile uint16_t throttle_pulse = 0;
volatile uint16_t steering_pulse = 0;
volatile uint16_t aux_pulse = 0;

// Timing variables for pulse measurement
volatile uint32_t throttle_start = 0;
volatile uint32_t steering_start = 0;
volatile uint32_t aux_start = 0;

// System timer counter
volatile uint32_t timer_counter = 0;

// Signal tracking variables
volatile uint32_t last_valid_signal_time = 0;
volatile uint32_t system_time = 0;
volatile bool connection_active = false;

// Function prototypes
void timer_init(void);
void pwm_outputs_init(void);
void rc_inputs_init(void);
void tm1637_init(void);
void tm1637_display(uint8_t segments[]);
void display_value(uint16_t value);
void update_outputs(uint16_t throttle, uint16_t steering, bool aux);

// Timer overflow interrupt (for microsecond counter)
ISR(TCA0_OVF_vect) {
    timer_counter += 65536;  // 16-bit overflow
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;  // Clear interrupt flag
}

// Pin change interrupt for RC pulse measurement
ISR(PORTD_PORT_vect) {
    uint32_t current_time = timer_counter + TCA0.SINGLE.CNT;
    uint8_t pin_state = PORTD.IN;
    
    // Throttle pin (PD2)
    if (PORTD.INTFLAGS & (1 << RC_THROTTLE_PIN)) {
        if (pin_state & (1 << RC_THROTTLE_PIN)) {
            // Rising edge
            throttle_start = current_time;
        } else {
            // Falling edge - calculate pulse width
            uint32_t pulse = current_time - throttle_start;
            // Only accept valid RC pulses (typically 1000-2000탎)
            if (pulse >= 900 && pulse <= 2100) {
                throttle_pulse = pulse;
                last_valid_signal_time = system_time;
                connection_active = true;
            }
        }
        // Clear interrupt flag
        PORTD.INTFLAGS = (1 << RC_THROTTLE_PIN);
    }
    
    // Steering pin (PD3)
    if (PORTD.INTFLAGS & (1 << RC_STEERING_PIN)) {
        if (pin_state & (1 << RC_STEERING_PIN)) {
            // Rising edge
            steering_start = current_time;
        } else {
            // Falling edge - calculate pulse width
            uint32_t pulse = current_time - steering_start;
            // Only accept valid RC pulses (typically 1000-2000탎)
            if (pulse >= 900 && pulse <= 2100) {
                steering_pulse = pulse;
                last_valid_signal_time = system_time;
                connection_active = true;
            }
        }
        // Clear interrupt flag
        PORTD.INTFLAGS = (1 << RC_STEERING_PIN);
    }
    
    // Aux pin (PD4)
    if (PORTD.INTFLAGS & (1 << RC_AUX_PIN)) {
        if (pin_state & (1 << RC_AUX_PIN)) {
            // Rising edge
            aux_start = current_time;
        } else {
            // Falling edge - calculate pulse width
            uint32_t pulse = current_time - aux_start;
            // Only accept valid RC pulses (typically 1000-2000탎)
            if (pulse >= 900 && pulse <= 2100) {
                aux_pulse = pulse;
                last_valid_signal_time = system_time;
                connection_active = true;
            }
        }
        // Clear interrupt flag
        PORTD.INTFLAGS = (1 << RC_AUX_PIN);
    }
}

int main(void) {
    // Set clock to 4MHz
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;
    
    // Initialize timer for pulse width measurement
    timer_init();
    
    // Initialize PWM outputs
    pwm_outputs_init();
    
    // Initialize RC inputs
    rc_inputs_init();
    
    // Setup output pins
    PORTD.DIRSET = (1 << LED_PIN);
    PORTD.OUTCLR = (1 << LED_PIN);  // LED off initially
    
    // Initialize display
    tm1637_init();
    
    // Display startup message
    display_value(8888);
    _delay_ms(500);
    
    // Display no connection initially
    display_value(DISPLAY_NOT_CONNECTED);
    
    // Enable global interrupts
    sei();
    
    // Main loop
    while(1) {
        // Update system time
        system_time += 20;
        
        // Check for signal loss
        if (connection_active && (system_time - last_valid_signal_time > 1000)) {
            // Connection lost for more than 1 second
            connection_active = false;
            
            // Safety measures
            update_outputs(1500, 1500, false);  // Neutral position
            
            // Update display to show no connection
            display_value(DISPLAY_NOT_CONNECTED);
        }
        
        // If connection is active, process RC signals
        if (connection_active) {
            // Convert aux pulse to boolean (on/off)
            bool aux_state = (aux_pulse > 1500);
            
            // Update outputs based on RC signals
            update_outputs(throttle_pulse, steering_pulse, aux_state);
            
            // Update display to show connection status
            display_value(DISPLAY_CONNECTED);
        } else {
            // No valid signal - blink LED as indicator
            PORTD.OUTTGL = (1 << LED_PIN);
        }
        
        // Delay between updates (50Hz is standard for RC systems)
        _delay_ms(20);
    }
    
    return 0;
}

// Initialize timer for microsecond timing
void timer_init(void) {
    // Configure TCA0 for microsecond timing (1탎 per tick)
    // TCA_SINGLE_CLKSEL_DIV4_gc = 0x04 (division by 4)
    // TCA_SINGLE_WGMODE_NORMAL_gc = 0x00 (normal mode)
    TCA0.SINGLE.CTRLA = 0x04;  // Division by 4 (4MHz / 4 = 1MHz)
    TCA0.SINGLE.CTRLB = 0x00;  // Normal mode
    TCA0.SINGLE.PER = 0xFFFF;  // Maximum period
    
    // Enable overflow interrupt
    // TCA_SINGLE_OVF_bm = 0x01
    TCA0.SINGLE.INTCTRL = 0x01;
    
    // Enable timer
    // TCA_SINGLE_ENABLE_bm = 0x01
    TCA0.SINGLE.CTRLA |= 0x01;
}

// Initialize PWM outputs for motor and servo
void pwm_outputs_init(void) {
    // Set pins as outputs
    PORTA.DIRSET = (1 << MOTOR_PIN) | (1 << SERVO_PIN);
    
    // Configure TCB0 for motor PWM output - using direct values
    // TCB_CLKSEL_DIV1_gc = 0x00 (no division)
    // TCB_CNTMODE_PWM8_gc = 0x07 (8-bit PWM mode)
    // TCB_CCMPEN_bm = 0x10 (Compare/Capture output enabled)
    TCB0.CTRLA = 0x00;  // No clock division
    TCB0.CTRLB = 0x17;  // 8-bit PWM mode with output enabled
    TCB0.CCMP = 0x8000;  // Start at 50% duty cycle
    
    // TCB_ENABLE_bm = 0x01
    TCB0.CTRLA |= 0x01;  // Enable timer
    
    // Configure TCB1 for servo PWM output - using direct values
    TCB1.CTRLA = 0x00;  // No clock division
    TCB1.CTRLB = 0x17;  // 8-bit PWM mode with output enabled
    TCB1.CCMP = 0x8000;  // Start at 50% duty cycle
    
    // TCB_ENABLE_bm = 0x01
    TCB1.CTRLA |= 0x01;  // Enable timer
}

// Initialize RC input pins with pin change interrupts
void rc_inputs_init(void) {
    // Set RC input pins as inputs with pull-ups
    PORTD.DIRCLR = (1 << RC_THROTTLE_PIN) | (1 << RC_STEERING_PIN) | (1 << RC_AUX_PIN);
    
    // PORT_ISC_BOTHEDGES_gc = 0x01, PORT_PULLUPEN_bm = 0x08
    PORTD.PIN2CTRL = 0x09;  // Both edges interrupt with pullup
    PORTD.PIN3CTRL = 0x09;  // Both edges interrupt with pullup
    PORTD.PIN4CTRL = 0x09;  // Both edges interrupt with pullup
}

// Update output signals based on RC inputs
void update_outputs(uint16_t throttle, uint16_t steering, bool aux) {
    // Motor control (throttle) - map 1000-2000탎 to 0-255 PWM
    uint8_t motor_pwm = (throttle - 1000) * 255 / 1000;
    if (motor_pwm > 255) motor_pwm = 255;
    
    // Servo control (steering) - convert to appropriate PWM signal
    // Scale 1000-2000탎 to 1000-2000탎 (same range, but might need adjustment)
    uint16_t servo_pwm = steering;
    
    // Set motor PWM
    TCB0.CCMP = ((uint16_t)motor_pwm << 8) | motor_pwm;
    
    // Set servo PWM - convert to timer ticks
    uint16_t servo_ticks = servo_pwm * 2;  // At 2MHz timer, 1탎 = 2 ticks
    TCB1.CCMP = ((uint16_t)servo_ticks << 8) | 0xFF;
    
    // Set LED based on aux channel
    if (aux) {
        PORTD.OUTSET = (1 << LED_PIN);  // LED on
    } else {
        PORTD.OUTCLR = (1 << LED_PIN);  // LED off
    }
}

// ===== TM1637 Display Functions =====
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
    for(uint8_t i = 0; i < 8; i++) {
        TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        
        if(b & 0x01)
            TM1637_PORT.OUTSET = (1 << TM1637_DIO_PIN);
        else
            TM1637_PORT.OUTCLR = (1 << TM1637_DIO_PIN);
        
        _delay_us(2);
        TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        b >>= 1;
    }
    
    // Wait for ACK
    TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
    TM1637_PORT.DIRCLR = (1 << TM1637_DIO_PIN);  // Set DIO as input
    _delay_us(5);
    
    TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
    
    TM1637_PORT.DIRSET = (1 << TM1637_DIO_PIN);  // Set DIO back to output
    _delay_us(2);
}

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

void tm1637_display(uint8_t segments[]) {
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_ADDR);
    
    for(uint8_t i = 0; i < 4; i++) {
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
    for(uint8_t i = 0; i < 4; i++) {
        segments[i] = digitToSegment[digits[i]];
    }
    
    if(value > 0) {
        if(value < 1000) segments[0] = 0;
        if(value < 100) segments[1] = 0;
        if(value < 10) segments[2] = 0;
    }
    
    tm1637_display(segments);
}