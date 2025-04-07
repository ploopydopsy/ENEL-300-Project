#define F_CPU 4000000UL  // Change to match actual clock frequency
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

//======== PIN ASSIGNMENTS ========//

// TM1637 display pins
#define TM1637_CLK_PIN 7            // PD7
#define TM1637_DIO_PIN 6            // PD6
#define TM1637_PORT PORTD           // PORTD for TM1637

// HC-SR04 sensor pins
#define HC_SR04_TRIG_PIN 1          // PD1
#define HC_SR04_ECHO_PIN 4          // PD4

// Motor control pins
#define MOTOR_ENA_PIN 1             // PA1 (PWM output)
#define MOTOR_IN1_PIN 2             // PA2
#define MOTOR_IN2_PIN 3             // PA3
#define MOTOR_PORT PORTA            // PORTA for motor control

// Joystick pin
#define JOYSTICK_PIN 2              // PD2 (AIN2)
#define JOYSTICK_PORT PORTD         // PORTD for joystick

// Display settings
#define TM1637_BRIGHTNESS 0x0F      // Maximum brightness (0x08 to 0x0F)
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88 // Display ON with brightness

//CONSTANTS
#define FILTER_SIZE 5
#define DEADZONE_LOW 115            // Lower deadzone threshold
#define DEADZONE_HIGH 132           // Upper deadzone threshold

// Timer variables for echo measurement
volatile uint16_t echo_start = 0;
volatile uint16_t echo_end = 0;
volatile uint8_t echo_done = 0;

uint16_t distance_buffer[FILTER_SIZE] = {0};
uint8_t buffer_index = 0;
uint8_t buffer_filled = 0;


//======== TM1637 DISPLAY FUNCTIONS ========//

// 7-segment lookup for digits 0-9
const uint8_t digitToSegment[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t b);
void tm1637_init(void);
void tm1637_display(uint8_t segments[]);
void display_value(uint16_t value);

void init_pins(void) {
    // Configure TM1637 pins as outputs FIRST
    TM1637_PORT.DIRSET = (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    
    // Configure motor control pins as outputs
    MOTOR_PORT.DIRSET = (1 << MOTOR_ENA_PIN) | (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    
    // Configure joystick pin as input
    JOYSTICK_PORT.DIRCLR = (1 << JOYSTICK_PIN);
    
    // Initialize motor outputs LOW
    MOTOR_PORT.OUTCLR = (1 << MOTOR_ENA_PIN) | (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    
    // HC-SR04 pins setup
    PORTD.DIRSET = (1 << HC_SR04_TRIG_PIN);     // TRIG as output
    PORTD.DIRCLR = (1 << HC_SR04_ECHO_PIN);     // ECHO as input
    PORTD.OUTCLR = (1 << HC_SR04_TRIG_PIN);     // Initial TRIG state low
    
    // Configure pin interrupt for ECHO pin
    PORTD.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
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

//======== JOYSTICK FUNCTIONS ========//

void adc_init(void) {
    SREG = 0b10000000;              // Enable global interrupts
    VREF.ADC0REF = 0b10000101;      // Set ADC reference to VDD
    ADC0.INTCTRL = 0b00000001;      // Enable ADC interrupt flag polling
    ADC0.MUXPOS = JOYSTICK_PIN;     // Use PD2 (AIN2)
    ADC0.CTRLC = 0x00;              // Minimum ADC clock division
    ADC0.CTRLA = 0b00000011;        // 12-bit resolution, free-running mode
    ADC0.COMMAND = 0x01;            // Start conversion
}

uint16_t readJoystick(void) {
    while (!(ADC0.INTFLAGS & 0x01)) { ; }  // Wait for conversion complete
    uint16_t value = ADC0.RES;
    ADC0.INTFLAGS = 0x01;                  // Clear ADC flag
    
    // Scale down to 0-255 range (8-bit)
    uint8_t scaled_value = (uint8_t)((value * 255UL) / 4095UL);
    
    return scaled_value;
}

// ======== MOTOR FUNCTIONS ========//

void init_pwm(void) {
    // Explicitly set PORTMUX to route TCA0 to PORTA
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc; 
    
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm; // Use CMP1 for PA1
    TCA0.SINGLE.PER = 255;  // 8-bit resolution
    TCA0.SINGLE.CMP1 = 0;   // Start with 0% duty (motor off)
}

void controlMotor(uint16_t joystickADC) {
    uint8_t duty = 0;
    
    // Debug output - apply a simple fixed duty cycle first
    // TCA0.SINGLE.CMP1 = 128; // 50% motor speed
    // return;
    
    if (joystickADC < DEADZONE_LOW) {
        // Reverse
        MOTOR_PORT.OUTSET = (1 << MOTOR_IN1_PIN);  // IN1 = HIGH
        MOTOR_PORT.OUTCLR = (1 << MOTOR_IN2_PIN);  // IN2 = LOW
        // Scale [0 ? deadzoneLow] to [255 ? 0]
        duty = (DEADZONE_LOW - joystickADC) * 255 / DEADZONE_LOW;
    }
    else if (joystickADC > DEADZONE_HIGH) {
        // Forward
        MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN);  // IN1 = LOW
        MOTOR_PORT.OUTSET = (1 << MOTOR_IN2_PIN);  // IN2 = HIGH
        // Scale [deadzoneHigh ? 255] to [0 ? 255]
        duty = (joystickADC - DEADZONE_HIGH) * 255 / (255 - DEADZONE_HIGH);
    }
    else {
        // Deadzone (Stop)
        MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);  // IN1 = LOW, IN2 = LOW
        duty = 0;
    }
    
    // Set motor speed
    TCA0.SINGLE.CMP1 = duty;
}

// ======== MAIN FUNCTION ======== //
int main(void) {
    // Set clock to 4MHz to match F_CPU
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;
    
    // Initialize pins FIRST
    init_pins();
    
    // Initialize display
    tm1637_init();
    
    // Display test pattern to verify display works
    uint8_t test_segments[4] = {0x7F, 0x7F, 0x7F, 0x7F}; // All segments on
    tm1637_display(test_segments);
    _delay_ms(1000);
    
    // Initialize other components
    adc_init();
    init_pwm();
    
    sei();  // Enable interrupts
    
    // Display initial zero
    display_value(0);
    _delay_ms(500);
    
    while (1) {
        // Read joystick value
        uint8_t joystick = readJoystick();
        
        // Display the raw joystick value
        display_value(joystick);
        
        // Control motor based on joystick value
        controlMotor(joystick);
        
        // Add a short delay
        _delay_ms(100);
    }
}