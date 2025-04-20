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
 * - Distance display on TM1637 4-digit 7-segment display
 * 
 * The microcontroller receives control signals from:
 * - PD2: Metal detector servo control from ESP32 (digital HIGH/LOW)
 * 
 * Hardware Connections:
 * - PA0 (TCA0 WO0): Metal detector servo PWM signal (50Hz)
 * - PD1: HC-SR04 TRIG pin (10µs pulse output)
 * - PD4: HC-SR04 ECHO pin (input pulse duration measurement)
 * - PA3: TM1637 display CLK pin
 * - PA4: TM1637 display DIO pin
 * 
 * Key Features:
 * - 4MHz system clock derived from internal oscillator
 * - Precise distance measurement using timer capture
 * - Weighted average filtering for fast response while filtering glitches
 * - Startup animation on the display (8888 pattern)
 * - Bit-banged protocol implementation for TM1637 display
 * - Pin change interrupts for precise echo pulse timing
 * 
 * Distance Calculation:
 * Uses time-of-flight principle to calculate distance:
 * - Sends 10µs trigger pulse to HC-SR04
 * - Measures duration of echo pulse
 * - Converts duration to distance (d = duration ÷ 117)
 * - Applies filtering to reject outliers and smooth readings
 * 
 * Metal Detector Operation:
 * - When ESP32 signal on PD2 is HIGH: Servo moves to DOWN position
 * - When ESP32 signal on PD2 is LOW: Servo moves to UP position
 * - Uses PWM signals of ~1.1ms (UP) and ~1.9ms (DOWN) pulse width
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
#define TRIG_PIN 1               // PD1 - Trigger pin (changed from PA2)
#define ECHO_PIN 4               // PD4 - Echo pin (changed from PC0)

// TM1637 Display
#define CLK_PIN 3                // PA3 - Clock pin
#define DIO_PIN 4                // PA4 - Data pin

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

//======== TM1637 DISPLAY CONSTANTS ========//

#define TM1637_BRIGHTNESS 0x0F   // Maximum brightness (0x08 to 0x0F)
#define TM1637_CMD_SET_DATA 0x40 // Command byte for data settings
#define TM1637_CMD_SET_ADDR 0xC0 // Command byte for address settings
#define TM1637_CMD_DISPLAY_CTRL 0x88 // Command byte for display control

// 7-segment lookup for digits 0-9
const uint8_t digit_to_segment[10] = {
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

// TM1637 Display functions
void tm1637_init(void);
void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t data);
void tm1637_display(uint8_t segments[]);
void display_distance(uint16_t distance);
void display_startup_animation(void);

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
    // PORTA - Configure outputs for servo, TM1637
    PORTA.DIRSET = (1 << METAL_SERVO_PIN) | (1 << CLK_PIN) | (1 << DIO_PIN);
    
    // Set initial states
    PORTA.OUTSET = (1 << CLK_PIN);       // CLK pin HIGH initially
    PORTA.OUTSET = (1 << DIO_PIN);       // DIO pin HIGH initially
    
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

//======== TM1637 DISPLAY FUNCTIONS ========//

void tm1637_init(void) {
    _delay_ms(50);  // Allow display to initialize
    
    // Set data command (auto address increment)
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_DATA);
    tm1637_stop();
    
    // Set display control (display on, max brightness)
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_DISPLAY_CTRL | TM1637_BRIGHTNESS);
    tm1637_stop();
    
    // Clear display initially
    uint8_t clear_segments[4] = {0, 0, 0, 0};
    tm1637_display(clear_segments);
}

void display_startup_animation(void) {
    // Show all segments lit (8888)
    uint8_t all_on[4] = {0x7F, 0x7F, 0x7F, 0x7F};
    tm1637_display(all_on);
    _delay_ms(1500);  // Show a bit longer
    
    // Blank display briefly before operation
    uint8_t segments[4] = {0, 0, 0, 0};
    tm1637_display(segments);
    _delay_ms(300);
}

void tm1637_start(void) {
    // Communication start condition
    PORTA.OUTSET = (1 << DIO_PIN);
    PORTA.OUTSET = (1 << CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << DIO_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << CLK_PIN);
    _delay_us(2);
}

void tm1637_stop(void) {
    // Communication stop condition
    PORTA.OUTCLR = (1 << CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << DIO_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << CLK_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << DIO_PIN);
    _delay_us(2);
}

void tm1637_write_byte(uint8_t data) {
    // Send 8 bits LSB first
    for (uint8_t i = 0; i < 8; i++) {
        PORTA.OUTCLR = (1 << CLK_PIN);
        _delay_us(2);
        
        if (data & 0x01) {
            PORTA.OUTSET = (1 << DIO_PIN);
        } else {
            PORTA.OUTCLR = (1 << DIO_PIN);
        }
        
        _delay_us(2);
        PORTA.OUTSET = (1 << CLK_PIN);
        _delay_us(2);
        data >>= 1;
    }
    
    // Wait for ACK
    PORTA.OUTCLR = (1 << CLK_PIN);
    PORTA.DIRCLR = (1 << DIO_PIN);  // Set DIO as input for ACK
    _delay_us(5);
    
    // Read ACK (but we don't use it)
    PORTA.OUTSET = (1 << CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << CLK_PIN);
    
    PORTA.DIRSET = (1 << DIO_PIN);  // Set DIO back to output
    _delay_us(2);
}

void tm1637_display(uint8_t segments[]) {
    // Start communication and set address command
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_ADDR);
    
    // Send each segment data
    for (uint8_t i = 0; i < 4; i++) {
        tm1637_write_byte(segments[i]);
    }
    
    // End communication
    tm1637_stop();
}

void display_distance(uint16_t distance) {
    // Split distance into individual digits
    uint8_t thousands = (distance / 1000) % 10;
    uint8_t hundreds = (distance / 100) % 10;
    uint8_t tens = (distance / 10) % 10;
    uint8_t ones = distance % 10;
    
    // Convert to segments
    uint8_t segments[4];
    segments[0] = thousands ? digit_to_segment[thousands] : 0;
    segments[1] = (hundreds || thousands) ? digit_to_segment[hundreds] : 0;
    segments[2] = (tens || hundreds || thousands) ? digit_to_segment[tens] : 0;
    segments[3] = digit_to_segment[ones];
    
    // Always show at least one digit
    if (distance == 0) {
        segments[3] = digit_to_segment[0];
    }
    
    // Display the segments
    tm1637_display(segments);
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
    tm1637_init();
    
    // Run startup animation
    display_startup_animation();
    
    // Display initial 0 on display
    display_distance(0);
    
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
            display_distance(distance_cm);
        }
        
        // 5. Shorter delay between measurements for faster response
        _delay_ms(30);
    }
    
    return 0;
}