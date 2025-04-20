#define F_CPU 4000000UL
// Integrated distance sensor with independently controlled servo

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// TM1637 display pins
#define TM1637_CLK_PIN 2  // PA2
#define TM1637_DIO_PIN 3  // PA3

// HC-SR04 sensor pins - moved to PORTA
#define HC_SR04_TRIG_PIN 1  // PA1
#define HC_SR04_ECHO_PIN 0  // PA0

// Servo pin
#define SERVO_PIN 0  // PC0

// ADC Input pin
#define ADC_INPUT_PIN 4  // PD4

// Display settings
#define TM1637_BRIGHTNESS 0x0F  // Maximum brightness (0x08 to 0x0F)
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88  // Display ON with brightness

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

// Timer variables for echo measurement
volatile uint16_t echo_start = 0;
volatile uint16_t echo_end = 0;
volatile uint8_t echo_done = 0;

// Moving average filter variables
#define FILTER_SIZE 5
uint16_t distance_buffer[FILTER_SIZE] = {0};
uint8_t buffer_index = 0;
uint8_t buffer_filled = 0;

// Function prototypes
void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t b);
void tm1637_init(void);
void tm1637_display(uint8_t segments[]);

void timer_init(void);
void trigger_measurement(void);
uint16_t calculate_distance(void);
uint16_t apply_moving_average(uint16_t new_distance);
void display_distance(uint16_t distance);

void servo_init(void);
void set_servo_position(uint16_t adc_value);

void adc_init(void);
uint16_t read_adc(void);

// Timer overflow interrupt
ISR(TCB0_INT_vect) {   
    // This interrupt will trigger if the echo is too long
    // Reset everything for the next measurement
    TCB0.INTFLAGS = TCB_OVF_bm; // Clear the interrupt flag
    echo_done = 1;
}

// Pin change interrupt for echo pin - moved to PORTA
ISR(PORTA_PORT_vect) {
    // Check if it's the echo pin
    if (PORTA.INTFLAGS & (1 << HC_SR04_ECHO_PIN)) {
        // If echo pin is high, start the timer
        if (PORTA.IN & (1 << HC_SR04_ECHO_PIN)) {
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
        PORTA.INTFLAGS = (1 << HC_SR04_ECHO_PIN); // Clear the interrupt flag
    }
}

int main(void) {
    // Set clock to 4MHz using internal high-frequency oscillator with prescaler
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;
    
    // Setup TM1637 pins (PA2 for CLK, PA3 for DIO) as outputs and set high
    PORTA.DIRSET = (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    
    // Setup HC-SR04 sensor pins: TRIG (PA1) as output, ECHO (PA0) as input
    PORTA.DIRSET = (1 << HC_SR04_TRIG_PIN);
    PORTA.DIRCLR = (1 << HC_SR04_ECHO_PIN);
    PORTA.OUTCLR = (1 << HC_SR04_TRIG_PIN); // Ensure TRIG is initially low
    
    // Configure pin interrupt for ECHO pin
    PORTA.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc; // Pull-up enabled, interrupt on both edges
    
    // Setup ADC input pin PD4
    PORTD.DIRCLR = (1 << ADC_INPUT_PIN);  // Set as input
    
    // Initialize timer for echo measurement
    timer_init();
    
    // Initialize display
    tm1637_init();
    
    // Initialize servo
    servo_init();
    
    // Initialize ADC
    adc_init();
    
    // Allow sensor to settle on startup
    _delay_ms(500);
    
    // Enable global interrupts
    sei();
    
    uint16_t distance_cm;
    uint16_t adc_value;
    
    // Display zero initially
    display_distance(0);
    
    while(1) {
        // Trigger a new distance measurement
        trigger_measurement();
        
        // Wait for the echo measurement to complete
        while(!echo_done);
        
        // Calculate distance in cm
        distance_cm = calculate_distance();
        
        // Apply moving average filter
        distance_cm = apply_moving_average(distance_cm);
        
        // Display the filtered distance
        display_distance(distance_cm);
        
        // Read analog value for servo control (independent of distance)
        adc_value = read_adc();
        
        // Update servo position based on ADC value
        set_servo_position(adc_value);
        
        // Reset for next measurement
        echo_done = 0;
        
        // Delay between measurements
        _delay_ms(100);
    }
    
    return 0;
}

void timer_init(void) {
    // Configure TCB0 for microsecond timing
    // At 4MHz, with prescaler 2, each tick is 0.5us
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc;    // Select DIV2 prescaler
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;    // Select periodic interrupt mode
    TCB0.CCMP = 60000;                  // Set overflow value (30ms timeout)
    TCB0.INTCTRL = TCB_OVF_bm;          // Enable overflow interrupt
    // Timer is enabled in the ISR when echo pin goes high
}

void adc_init(void) {
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;  // Set ADC reference to VDD
    ADC0.CTRLC = ADC_PRESC_DIV4_gc;     // Set ADC clock prescaler
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_12BIT_gc;  // Enable ADC, 12-bit resolution
    ADC0.MUXPOS = ADC_MUXPOS_AIN4_gc;   // Select PD4 (AIN4) as input
}

uint16_t read_adc(void) {
    ADC0.COMMAND = ADC_STCONV_bm;  // Start conversion
    
    while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));  // Wait for conversion to complete
    
    uint16_t result = ADC0.RES;  // Read result
    ADC0.INTFLAGS = ADC_RESRDY_bm;  // Clear the result ready flag
    
    return result;
}

void trigger_measurement(void) {
    // Make sure timer is stopped and interrupts are cleared
    TCB0.CTRLA &= ~TCB_ENABLE_bm;
    TCB0.INTFLAGS = TCB_OVF_bm;
    PORTA.INTFLAGS = (1 << HC_SR04_ECHO_PIN);
    
    // Send a 10us pulse to TRIG pin
    PORTA.OUTSET = (1 << HC_SR04_TRIG_PIN);
    _delay_us(10);
    PORTA.OUTCLR = (1 << HC_SR04_TRIG_PIN);
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
    // To avoid floating point: Distance = pulse_duration ÷ 116.6 (approx)
    
    uint16_t distance = pulse_duration / 117;
    
    // Limit max distance to 400cm (HC-SR04 max range)
    if (distance > 400) {
        distance = 0; // Out of range
    }
    
    return distance;
}

uint16_t apply_moving_average(uint16_t new_distance) {
    // Add new distance to buffer
    distance_buffer[buffer_index] = new_distance;
    
    // Update buffer index
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    
    // Update buffer filled status
    if (buffer_index == 0) {
        buffer_filled = 1;
    }
    
    // Calculate average
    uint32_t sum = 0;
    uint8_t count = buffer_filled ? FILTER_SIZE : buffer_index;
    
    for (uint8_t i = 0; i < count; i++) {
        sum += distance_buffer[i];
    }
    
    // Return average (prevent division by zero)
    return (count > 0) ? (uint16_t)(sum / count) : 0;
}

void tm1637_init(void) {
    _delay_ms(50);  // Let display stabilize
    
    // Set data command (automatic address increment + normal mode)
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_DATA);
    tm1637_stop();
    
    // Set display ON with brightness
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_DISPLAY_CTRL | (TM1637_BRIGHTNESS & 0x07));
    tm1637_stop();
    
    // Clear display initially
    uint8_t clear_segments[4] = {0, 0, 0, 0};
    tm1637_display(clear_segments);
}

void tm1637_display(uint8_t segments[]) {
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_ADDR);  // Set starting address to 0
    
    for(uint8_t i = 0; i < 4; i++) {
        tm1637_write_byte(segments[i]);
    }
    
    tm1637_stop();
}

void tm1637_start(void) {
    PORTA.OUTSET = (1 << TM1637_DIO_PIN);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
}

void tm1637_stop(void) {
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}

void tm1637_write_byte(uint8_t b) {
    for(uint8_t i = 0; i < 8; i++) {
        PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        
        if(b & 0x01)
            PORTA.OUTSET = (1 << TM1637_DIO_PIN);
        else
            PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
        
        _delay_us(2);
        PORTA.OUTSET = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        b >>= 1;
    }
    
    // Wait for ACK
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTA.DIRCLR = (1 << TM1637_DIO_PIN);  // Set DIO as input for ACK
    _delay_us(5);
    
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    
    PORTA.DIRSET = (1 << TM1637_DIO_PIN);  // Set DIO back to output
    _delay_us(2);
}

void display_distance(uint16_t distance) {
    // Break the distance into 4 digits
    uint8_t digits[4];
    digits[0] = (distance / 1000) % 10;
    digits[1] = (distance / 100) % 10;
    digits[2] = (distance / 10) % 10;
    digits[3] = distance % 10;
    
    // Convert to segments
    uint8_t segments[4];
    for(uint8_t i = 0; i < 4; i++) {
        segments[i] = digitToSegment[digits[i]];
    }
    
    // Don't show leading zeros
    if(distance < 1000) segments[0] = 0;
    if(distance < 100) segments[1] = 0;
    if(distance < 10) segments[2] = 0;
    
    // Always show at least one digit
    if(distance == 0) segments[3] = digitToSegment[0];
    
    // Display the segments
    tm1637_display(segments);
}

void servo_init(void) {
    // Configure PC0 as output for servo PWM
    PORTC.DIRSET = (1 << SERVO_PIN);
    
    // Route TCA0 WO outputs to PORTC instead of default PORTA
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc;
    
    // Set up TCA0 for 50Hz PWM (20ms period)
    // At 4MHz with DIV8 prescaler, tick rate is 500kHz (2us per tick)
    // Period = 20ms / 2us = 10000 ticks
    TCA0.SINGLE.PER = 10000;
    
    // Enable compare channel 0 (WO0 on PC0) in single-slope PWM mode
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    
    // Start with middle position (~1.5ms pulse width = 750 ticks)
    TCA0.SINGLE.CMP0 = 750;
    
    // Start TCA0 with DIV8 prescaler
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
}

void set_servo_position(uint16_t adc_value) {
    // Map ADC value (0-4095) to servo pulse width (500-2500us)
    // Using 500kHz timer, pulse width in ticks: 250-1250
    
    // Map 0-4095 to 250-1250 ticks
    uint16_t servo_pulse = 250 + ((adc_value * 1000UL) / 4095UL);
    
    // Ensure pulse width stays within safe servo limits
    if (servo_pulse < 250) servo_pulse = 250;    // 500us minimum
    if (servo_pulse > 1250) servo_pulse = 1250;  // 2500us maximum
    
    // Update the compare value to set servo position
    TCA0.SINGLE.CMP0 = servo_pulse;
}