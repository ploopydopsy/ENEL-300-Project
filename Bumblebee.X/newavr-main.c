/**
 * File:   AVR_SECONDARY.c
 * Author: william adebiyi
 *
 * Created on April 20, 2025
 * 
 * Secondary AVR128DB28 Microcontroller for RC Car Project
 * ENEL 300 - Winter 2025
 * 
 * This secondary microcontroller handles the sensor and display aspects:
 * - Metal detector arm servo control (up/down position)
 * - Ultrasonic distance measurements (HC-SR04 sensor)
 * - Distance display on I2C LCD (bit-banged implementation)
 * 
 * The microcontroller receives control signals from:
 * - PD2: Metal detector servo control from ESP32 (digital HIGH/LOW)
 * 
 * Hardware Connections:
 * - PA0 (TCA0 WO0): Metal detector servo PWM signal (50Hz)
 * - PD1: HC-SR04 TRIG pin (10µs pulse output)
 * - PD4: HC-SR04 ECHO pin (input pulse duration measurement)
 * - PA3: I2C SCL pin (was TM1637 CLK)
 * - PA4: I2C SDA pin (was TM1637 DIO)
 */

#define F_CPU 4000000UL  // 4MHz clock

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>  // For abs() function
#include <stdbool.h> // For bool, true, false

//======== PIN DEFINITIONS ========//

// Metal Detector Servo
#define METAL_SERVO_PIN 0        // PA0 (TCA0 WO0) - Metal detector servo

// HC-SR04 Ultrasonic Sensor
#define TRIG_PIN 1               // PD1 - Trigger pin
#define ECHO_PIN 4               // PD4 - Echo pin

// I2C LCD pins (same as old TM1637 Display pins)
#define SCL_PIN 3                // PA3 - I2C Clock pin (was CLK_PIN)
#define SDA_PIN 4                // PA4 - I2C Data pin (was DIO_PIN)

// ESP32 Communication Pins
#define METAL_SERVO_INPUT_PIN 2  // PD2 - Metal detector control from ESP32 (Digital)

//======== SERVO CONTROL CONSTANTS ========//

// Pulse width values for TCA0 at 4MHz with prescaler 64 = 62.5kHz tick rate
// 62.5kHz ? 1 tick ? 16?s
#define SERVO_PERIOD 1250        // 20ms period (50Hz) = 1250 ticks
#define SERVO_UP_POSITION 70     // Servo up position (~1.1ms pulse)
#define SERVO_DOWN_POSITION 120  // Servo down position (~2.2ms pulse) [140]

//======== DISTANCE SENSOR CONSTANTS ========//

// For ultrasonic ranging
#define SOUND_SPEED_FACTOR 58    // Sound speed conversion factor for cm
#define MAX_DISTANCE 400         // Maximum measurable distance in cm
#define MIN_DISTANCE 2           // Minimum reliable distance in cm

//======== I2C LCD CONSTANTS ========//
#define LCD_I2C_ADDR 0x27        // Common I2C address for PCF8574 backpack (might be 0x3F)

// LCD commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// PCF8574 expander bits
#define LCD_BACKLIGHT 0x08
#define LCD_EN 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01

//======== GLOBAL VARIABLES ========//

// Distance measurement
volatile uint16_t echo_start = 0;
volatile uint16_t echo_end = 0;
volatile uint16_t pulse_duration = 0;
volatile bool echo_received = false;
uint16_t distance_cm = 0;

// Metal detector servo state
bool metal_detector_down = false;

// Moving average filter for distance readings
#define FILTER_SIZE 5
uint16_t distance_buffer[FILTER_SIZE] = {0};
uint8_t buffer_index = 0;

// LCD settings
uint8_t lcd_backlight = LCD_BACKLIGHT;  // Backlight on by default

//======== FUNCTION PROTOTYPES ========//

// Initialization functions
void init_clock(void);
void init_pins(void);
void init_servo_pwm(void);
void init_timer_capture(void);

// Distance sensor functions
void trigger_measurement(void);
uint16_t calculate_distance(uint16_t duration);
uint16_t filter_distance(uint16_t new_distance);

// Bit-banged I2C functions
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
bool i2c_write_byte(uint8_t data);
void i2c_send_cmd(uint8_t command);

// LCD functions
void lcd_init(void);
void lcd_send_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_byte(uint8_t byte, uint8_t rs);
void lcd_command(uint8_t command);
void lcd_data(uint8_t data);
void lcd_print(const char* str);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);
void lcd_display_distance(uint16_t distance);
void lcd_startup_animation(void);

// Metal detector servo control
void set_servo_position(bool down);

//======== INTERRUPT SERVICE ROUTINES ========//

// Pin change interrupt for echo pin
ISR(PORTD_PORT_vect) {
    // Check if it's the echo pin that triggered the interrupt
    if (PORTD.INTFLAGS & (1 << ECHO_PIN)) {
        // If echo pin is high, start the timer
        if (PORTD.IN & (1 << ECHO_PIN)) {
            TCB0.CNT = 0;          // Reset counter
            echo_start = TCB0.CNT; // Start time (should be 0)
            // Make sure timer is running
            TCB0.CTRLA |= TCB_ENABLE_bm;
        } 
        // If echo pin is low, calculate pulse duration
        else {
            echo_end = TCB0.CNT;
            pulse_duration = echo_end - echo_start;
            echo_received = true;
        }
        
        // Clear the interrupt flag for this pin
        PORTD.INTFLAGS = (1 << ECHO_PIN);
    }
}

//======== INITIALIZATION FUNCTIONS ========//

void init_clock(void) {
    // Configure for 4MHz using internal oscillator
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;        // Select internal high-frequency oscillator
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;  // 4x prescaler (16MHz/4 = 4MHz)
    
    // Wait for clock to stabilize
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm);
}

void init_pins(void) {
    // PORTA - Configure outputs for servo and I2C (SCL, SDA)
    PORTA.DIRSET = (1 << METAL_SERVO_PIN) | (1 << SCL_PIN) | (1 << SDA_PIN);
    
    // Set initial states for I2C
    PORTA.OUTSET = (1 << SCL_PIN);  // SCL HIGH initially
    PORTA.OUTSET = (1 << SDA_PIN);  // SDA HIGH initially
    
    // PORTD - Configure for TRIG (output), ECHO (input), and ESP32 input
    PORTD.DIRSET = (1 << TRIG_PIN);      // TRIG as output
    PORTD.DIRCLR = (1 << ECHO_PIN);      // ECHO as input
    PORTD.DIRCLR = (1 << METAL_SERVO_INPUT_PIN); // ESP32 input
    
    // Set initial TRIG state
    PORTD.OUTCLR = (1 << TRIG_PIN);      // TRIG pin LOW initially
    
    // Configure pin for interrupt (echo pin)
    PORTD.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;  // Pull-up enabled, interrupt on both edges
}

void init_servo_pwm(void) {
    // Configure TCA0 for servo PWM
    
    // For a 50Hz PWM with 4MHz clock and DIV64 prescaler:
    // Tick frequency = 4MHz/64 = 62.5kHz
    // For 50Hz output, period = 62.5kHz/50Hz = 1250 ticks
    TCA0.SINGLE.PER = SERVO_PERIOD;
    
    // Enable compare channel 0 (WO0 for PA0)
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
    
    // Set servo to up position initially
    TCA0.SINGLE.CMP0 = SERVO_UP_POSITION;
    
    // Start TCA0 with DIV64 prescaler
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
}

void init_timer_capture(void) {
    // Configure TCB0 for microsecond timing
    // At 4MHz, with prescaler 2, each tick is 0.5us
    
    // Reset variables
    echo_start = 0;
    echo_end = 0;
    echo_received = false;
    
    // Configure TCB0 as a simple counter (not capture mode)
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;    // Select periodic interrupt mode
    TCB0.CCMP = 60000;                  // Set overflow value (30ms timeout)
    
    // Don't enable interrupt - we just use TCB0 as a counter
    TCB0.INTCTRL = 0;
    
    // Start timer with DIV2 prescaler but don't enable yet
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc;  // Setup but don't enable
}

//======== DISTANCE SENSOR FUNCTIONS ========//

void trigger_measurement(void) {
    // Reset variables for new measurement
    echo_received = false;
    echo_start = 0;
    echo_end = 0;
    pulse_duration = 0;
    
    // Reset the timer
    TCB0.CNT = 0;
    
    // Make sure timer is ready but not running yet
    TCB0.CTRLA &= ~TCB_ENABLE_bm;
    
    // Clear any previous interrupt flags on PORTD
    PORTD.INTFLAGS = (1 << ECHO_PIN);
    
    // Send 10us trigger pulse
    PORTD.OUTSET = (1 << TRIG_PIN);
    _delay_us(10);
    PORTD.OUTCLR = (1 << TRIG_PIN);
    
    // Timer will be enabled in ISR when echo rises
}

uint16_t calculate_distance(uint16_t duration) {
    // Convert pulse duration to distance in cm
    // With 0.5us per tick: duration * 0.5 = pulse in us
    // Distance = pulse / 58 (for cm)
    
    // Check for invalid readings
    if (duration == 0 || duration > 35000) {  // >35000 ticks = >17.5ms = >3m
        return 0;  // Out of range or no echo
    }
    
    // Each tick is 0.5us at 4MHz with DIV2
    // Speed of sound is 343m/s = 34300cm/s = 0.0343cm/us
    // Distance = (time × speed of sound) ÷ 2
    // With 0.5us ticks: distance (cm) = duration × 0.5 × 0.0343 ÷ 2
    // Simplified: distance (cm) = duration ÷ 117
    
    uint16_t distance = duration / 117;
    
    // Limit to valid range
    if (distance < MIN_DISTANCE) {
        distance = MIN_DISTANCE;
    } else if (distance > MAX_DISTANCE) {
        distance = 0;  // Out of range
    }
    
    return distance;
}

uint16_t filter_distance(uint16_t new_distance) {
    // Quick rejection of obviously invalid readings
    static uint16_t last_distance = 0;
    
    // Only reject extremely large jumps (more than 100cm) for speed
    if (last_distance > 0 && new_distance > 0) {
        if (abs((int16_t)new_distance - (int16_t)last_distance) > 100) {
            return last_distance; // Keep previous reading for extreme jumps
        }
    }
    
    // Use a faster filter - weighted average favoring new readings
    // This responds faster than median while still smoothing
    static uint16_t filtered_distance = 0;
    
    if (filtered_distance == 0 && new_distance > 0) {
        // Initialize filter with first valid reading
        filtered_distance = new_distance;
    } else if (new_distance > 0) {
        // 70% new reading, 30% old reading - faster response
        filtered_distance = (new_distance * 7 + filtered_distance * 3) / 10;
    }
    
    // Update history
    last_distance = new_distance;
    
    return filtered_distance > 0 ? filtered_distance : new_distance;
}

//======== BIT-BANGED I2C FUNCTIONS ========//

void i2c_init(void) {
    // Initialize SDA and SCL as outputs
    PORTA.DIRSET = (1 << SCL_PIN) | (1 << SDA_PIN);
    PORTA.OUTSET = (1 << SCL_PIN);  // SCL HIGH
    PORTA.OUTSET = (1 << SDA_PIN);  // SDA HIGH
    _delay_ms(10);  // Startup delay
}

void i2c_start(void) {
    // START condition: SDA goes LOW while SCL is HIGH
    PORTA.OUTSET = (1 << SDA_PIN);  // SDA HIGH
    PORTA.OUTSET = (1 << SCL_PIN);  // SCL HIGH
    _delay_us(5);
    PORTA.OUTCLR = (1 << SDA_PIN);  // SDA LOW while SCL HIGH
    _delay_us(5);
    PORTA.OUTCLR = (1 << SCL_PIN);  // SCL LOW
    _delay_us(5);
}

void i2c_stop(void) {
    // STOP condition: SDA goes HIGH while SCL is HIGH
    PORTA.OUTCLR = (1 << SDA_PIN);  // SDA LOW
    _delay_us(5);
    PORTA.OUTSET = (1 << SCL_PIN);  // SCL HIGH
    _delay_us(5);
    PORTA.OUTSET = (1 << SDA_PIN);  // SDA HIGH while SCL HIGH
    _delay_us(5);
}

bool i2c_write_byte(uint8_t data) {
    uint8_t i;
    bool ack;
    
    // Send 8 bits, MSB first
    for (i = 0; i < 8; i++) {
        // Set SDA based on data bit
        if (data & 0x80)
            PORTA.OUTSET = (1 << SDA_PIN);  // SDA HIGH
        else
            PORTA.OUTCLR = (1 << SDA_PIN);  // SDA LOW
        
        data <<= 1;  // Shift left for next bit
        
        // Clock pulse
        _delay_us(5);
        PORTA.OUTSET = (1 << SCL_PIN);  // SCL HIGH
        _delay_us(5);
        PORTA.OUTCLR = (1 << SCL_PIN);  // SCL LOW
        _delay_us(5);
    }
    
    // Release SDA for slave to ACK
    PORTA.OUTSET = (1 << SDA_PIN);  // SDA HIGH (released)
    PORTA.DIRCLR = (1 << SDA_PIN);  // SDA as input
    
    // Clock pulse for ACK
    _delay_us(5);
    PORTA.OUTSET = (1 << SCL_PIN);  // SCL HIGH
    _delay_us(5);
    
    // Read ACK (LOW = ACK, HIGH = NACK)
    ack = !(PORTA.IN & (1 << SDA_PIN));
    
    PORTA.OUTCLR = (1 << SCL_PIN);  // SCL LOW
    _delay_us(5);
    
    // Set SDA back as output
    PORTA.DIRSET = (1 << SDA_PIN);
    PORTA.OUTCLR = (1 << SDA_PIN);  // SDA LOW
    
    return ack;
}

void i2c_send_cmd(uint8_t command) {
    i2c_start();
    i2c_write_byte(LCD_I2C_ADDR << 1);  // Address with write bit
    i2c_write_byte(command);
    i2c_stop();
}

//======== LCD FUNCTIONS ========//

void lcd_send_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble << 4) | lcd_backlight | rs;
    
    // Send with EN = 0
    i2c_start();
    i2c_write_byte(LCD_I2C_ADDR << 1);  // Address with write bit
    i2c_write_byte(data);
    i2c_stop();
    
    // Pulse EN (HIGH->LOW)
    i2c_start();
    i2c_write_byte(LCD_I2C_ADDR << 1);  // Address with write bit
    i2c_write_byte(data | LCD_EN);  // EN HIGH
    i2c_stop();
    
    _delay_us(1);  // Enable pulse must be >450ns
    
    i2c_start();
    i2c_write_byte(LCD_I2C_ADDR << 1);  // Address with write bit
    i2c_write_byte(data & ~LCD_EN);  // EN LOW
    i2c_stop();
    
    _delay_us(50);  // Commands need >37us to settle
}

void lcd_send_byte(uint8_t byte, uint8_t rs) {
    // Send high nibble first
    lcd_send_nibble(byte >> 4, rs);
    // Then low nibble
    lcd_send_nibble(byte & 0x0F, rs);
}

void lcd_command(uint8_t command) {
    // RS = 0 for command
    lcd_send_byte(command, 0);
}

void lcd_data(uint8_t data) {
    // RS = 1 for data
    lcd_send_byte(data, LCD_RS);
}

void lcd_init(void) {
    _delay_ms(50);  // Wait for LCD to power up
    
    // Special initialization sequence for 4-bit mode
    // First try
    lcd_send_nibble(0x03, 0);
    _delay_ms(5);
    
    // Second try
    lcd_send_nibble(0x03, 0);
    _delay_ms(5);
    
    // Third try
    lcd_send_nibble(0x03, 0);
    _delay_us(150);
    
    // Finally set to 4-bit interface
    lcd_send_nibble(0x02, 0);
    _delay_us(150);
    
    // Now we can use lcd_command()
    // Function set: 4-bit mode, 2 lines, 5x8 font
    lcd_command(0x28);
    
    // Display control: Display on, Cursor off, Blink off
    lcd_command(0x0C);
    
    // Clear display
    lcd_command(0x01);
    _delay_ms(2);  // Clear needs extra time
    
    // Entry mode set: Increment, no shift
    lcd_command(0x06);
}

void lcd_clear(void) {
    lcd_command(LCD_CLEARDISPLAY);
    _delay_ms(2);  // This command takes longer
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_print(const char* str) {
    while (*str) {
        lcd_data(*str++);
    }
}

void lcd_display_distance(uint16_t distance) {
    // Clear the first line
    lcd_set_cursor(0, 0);
    lcd_print("Distance:      ");
    
    // Display distance value
    lcd_set_cursor(0, 10);
    
    if (distance == 0) {
        // Invalid or out of range
        lcd_print("---");
    } else {
        // Convert distance to string
        char buffer[8];
        uint8_t i = 0;
        uint16_t temp = distance;
        
        // Handle 0 specially
        if (temp == 0) {
            buffer[i++] = '0';
        } else {
            // Extract digits in reverse order
            while (temp > 0) {
                buffer[i++] = '0' + (temp % 10);
                temp /= 10;
            }
        }
        
        // Print digits in correct order
        while (i > 0) {
            lcd_data(buffer[--i]);
        }
        
        // Add "cm" unit
        lcd_print(" cm");
    }
}

void lcd_startup_animation(void) {
    // Show initial message
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("RC Car System");
    lcd_set_cursor(1, 0);
    lcd_print("Initializing...");
    
    _delay_ms(1000);
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Distance: 0 cm");
}

//======== METAL DETECTOR SERVO CONTROL ========//

void set_servo_position(bool down) {
    if (down) {
        TCA0.SINGLE.CMP0 = SERVO_DOWN_POSITION;
    } else {
        TCA0.SINGLE.CMP0 = SERVO_UP_POSITION;
    }
}

//======== MAIN FUNCTION ========//

int main(void) {
    // System initialization
    init_clock();
    init_pins();
    init_servo_pwm();
    init_timer_capture();
    
    // Initialize I2C and LCD
    i2c_init();
    lcd_init();
    
    // Run startup animation
    lcd_startup_animation();
    
    // Display initial 0 on display
    lcd_display_distance(0);
    
    // Enable global interrupts
    sei();
    
    // Main control loop
    while (1) {
        // 1. Check metal detector control from ESP32
        bool metal_input = (PORTD.IN & (1 << METAL_SERVO_INPUT_PIN)) > 0;
        
        // Update servo position if command has changed
        if (metal_input != metal_detector_down) {
            metal_detector_down = metal_input;
            set_servo_position(metal_detector_down);
        }
        
        // 2. Trigger a new distance measurement
        trigger_measurement();
        
        // 3. Wait for echo with short timeout (30ms max)
        uint8_t timeout_counter = 0;
        while (!echo_received && timeout_counter < 30) {
            _delay_ms(1);
            timeout_counter++;
        }
        
        // 4. Calculate and filter distance if echo received
        if (echo_received) {
            uint16_t raw_distance = calculate_distance(pulse_duration);
            distance_cm = filter_distance(raw_distance);
            
            // Always update display with latest filtered value
            lcd_display_distance(distance_cm);
        }
        
        // 5. Shorter delay between measurements for faster response
        _delay_ms(30);
    }
    
    return 0;
}