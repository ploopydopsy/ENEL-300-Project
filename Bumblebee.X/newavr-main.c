#define F_CPU 20000000UL  // Intended 20 MHz, but if your clock is actually 8MHz, this mapping uses a 1MHz tick (8MHz/8)

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// -----------------------
// Function Prototypes
// -----------------------
void adc_init(void);
uint16_t read_adc(void);

void servo_init(void);
void manual_servo_test(void);  // Uncomment to test PWM manually

// -----------------------
// Main
// -----------------------
int main(void) {
    // --- CLOCK CONFIGURATION (Intended 20 MHz) ---
    CCP = 0xD8;  // Unlock protected registers
    CLKCTRL.OSCHFCTRLA = 0b00010100;  // Set internal oscillator to 20 MHz
    while (CLKCTRL.MCLKSTATUS & 0x01) { ; }  // Wait for clock to stabilize

    // --- SERVO PWM INIT (TCA0 on PA0: WO0) ---
    servo_init();
    // Configure PA0 as output for servo PWM
    PORTA.DIRSET = (1 << 0);  // PA0

    // --- ADC INIT (using PD2 as ADC input) ---
    PORTD.DIRCLR = (1 << 2);  // PD2 as input
    adc_init();

    _delay_ms(500);  // Allow peripherals to settle

    // Uncomment the next line to manually cycle the PWM value for testing:
    // manual_servo_test();

    // Main loop: update servo PWM based on ADC reading.
    while (1) {
        uint16_t adc_value = read_adc();  // Expected range roughly 0 to 4095

        // Map ADC reading to servo pulse width:
        // For an 8MHz clock with DIV8, tick frequency is 1MHz.
        // For 50Hz PWM, set period = 20000 ticks (20ms).
        // Here we map from 500 ticks (0.5ms) to 2500 ticks (2.5ms).
        uint16_t servoPulse = 500 + ((adc_value * 2000UL) / 4095UL);
        TCA0.SINGLE.CMP0 = servoPulse;

        _delay_ms(100);
    }
    
    return 0;
}

// -----------------------
// ADC Functions
// -----------------------
void adc_init(void) {
    SREG = 0b10000000;              // Enable global interrupts
    VREF.ADC0REF = 0b10000101;        // Set ADC reference to VDD
    ADC0.INTCTRL = 0b00000001;        // Enable ADC interrupt flag polling
    ADC0.MUXPOS = 0x02;               // Use PD2 (AIN2)
    ADC0.CTRLC = 0x00;                // Minimum ADC clock division
    ADC0.CTRLA = 0b00000011;          // 12-bit resolution, free-running mode
    ADC0.COMMAND = 0x01;              // Start conversion
}

uint16_t read_adc(void) {
    while (!(ADC0.INTFLAGS & 0x01)) { ; }  // Wait for conversion complete
    uint16_t value = ADC0.RES;
    ADC0.INTFLAGS = 0x01;                  // Clear ADC flag
    return value;
}

// -----------------------
// Servo (PWM) Functions
// -----------------------
void servo_init(void) {
    // For a 50Hz PWM at 1MHz tick (8MHz clock with DIV8):
    // Period = 20ms * 1e6 = 20000 ticks.
    TCA0.SINGLE.PER = 20000;
    // Enable compare channel 0 (WO0 on PA0) in single-slope PWM mode.
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    // Start with a pulse width of ~1ms (1000 ticks).
    TCA0.SINGLE.CMP0 = 1000;
    // Start TCA0 with DIV8 prescaler.
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
}

// Uncomment and call this function to manually test PWM movement.
// It cycles the PWM pulse from 1ms to 2ms.
void manual_servo_test(void) {
    while(1) {
        for (uint16_t pwm = 1000; pwm <= 2000; pwm += 100) {
            TCA0.SINGLE.CMP0 = pwm;
            _delay_ms(500);
        }
        for (uint16_t pwm = 2000; pwm >= 1000; pwm -= 100) {
            TCA0.SINGLE.CMP0 = pwm;
            _delay_ms(500);
        }
    }
}