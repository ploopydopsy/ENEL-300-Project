#define F_CPU 20000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Function Prototypes
void adc_init(void);
uint16_t read_adc(uint8_t channel);
void servo_init(void);

// -----------------------
// Main
// -----------------------
int main(void) {
    // Clock Configuration
    CCP = 0xD8;  // Unlock protected registers
    CLKCTRL.OSCHFCTRLA = 0b00010100;  // Set internal oscillator to 20 MHz
    while (CLKCTRL.MCLKSTATUS & 0x01) { ; }  // Wait for clock to stabilize

    // Servo PWM INIT
    servo_init();
    PORTA.DIRSET = (1 << 0);  // PA0 Steering Servo
    PORTC.DIRSET = (1 << 0);  // PC0 Metal Detector Servo

    // ADC INIT (using PD2 and PD4 as ADC inputs)
    PORTD.DIRCLR = (1 << 2) | (1 << 4);  // PD2 and PD4 as inputs
    adc_init();

    _delay_ms(500);  // Allow peripherals to settle

    // Variables for filtering
    uint32_t filtered_adc_steering = 0;
    uint32_t filtered_adc_metal = 0;
    uint16_t prev_steering_servo_adc = 0;
    uint16_t prev_metal_servo_adc = 0;
    
    // Main loop: update servos
    while (1) {
        // Read ADC for Steering Servo (PD2)
        uint16_t adc_steering = read_adc(2);  // AIN2 on PD2
        
        // Read ADC for Metal Detector Servo (PD4)
        uint16_t adc_metal = read_adc(4);  // AIN4 on PD4
        
        // Apply light filtering (50% old value, 50% new value)
        filtered_adc_steering = (filtered_adc_steering + adc_steering) / 2;
        filtered_adc_metal = (filtered_adc_metal + adc_metal) / 2;

        // Update Steering Servo with minimal threshold
        if (filtered_adc_steering > prev_steering_servo_adc + 5 || 
            filtered_adc_steering < prev_steering_servo_adc - 5) {
            uint16_t servoPulse = 500 + ((filtered_adc_steering * 2000UL) / 4095UL);
            TCA0.SINGLE.CMP0 = servoPulse;  // Steering Servo (PA0)
            prev_steering_servo_adc = filtered_adc_steering;
        }

        // Update Metal Detector Servo with minimal threshold
        if (filtered_adc_metal > prev_metal_servo_adc + 5 || 
            filtered_adc_metal < prev_metal_servo_adc - 5) {
            uint16_t servoPulse = 500 + ((filtered_adc_metal * 2000UL) / 4095UL);
            TCA0.SINGLE.CMP1 = servoPulse;  // Metal Detector Servo (PC0)
            prev_metal_servo_adc = filtered_adc_metal;
        }

        _delay_ms(10);  // Update 100 times per second for minimal latency
    }
    
    return 0;
}

// ADC Functions
void adc_init(void) {
    SREG = 0b10000000;              // Enable global interrupts
    VREF.ADC0REF = 0b10000101;      // Set ADC reference to VDD
    ADC0.INTCTRL = 0b00000001;      // Enable ADC interrupt flag polling
    ADC0.CTRLC = 0x00;              // Minimum ADC clock division
    ADC0.CTRLA = 0b00000011;        // 12-bit resolution, free-running mode
}

uint16_t read_adc(uint8_t channel) {
    // Set ADC channel
    ADC0.MUXPOS = channel;
    
    // Start conversion
    ADC0.COMMAND = 0x01;
    
    // Wait for conversion complete
    while (!(ADC0.INTFLAGS & 0x01)) { ; }
    
    // Read value
    uint16_t value = ADC0.RES;
    
    // Clear interrupt flag
    ADC0.INTFLAGS = 0x01;
    
    return value;
}

// Servo (PWM) Functions
void servo_init(void) {
    // For a 50Hz PWM at 1MHz tick (8MHz clock with DIV8):
    // Period = 20ms * 1e6 = 20000 ticks.
    TCA0.SINGLE.PER = 20000;
    
    // Route TCA0 WO outputs to PORTA and PORTC
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc;
    
    // Enable compare channels for both servos in single-slope PWM mode
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm |   // Steering Servo (WO0)
                        TCA_SINGLE_CMP1EN_bm |   // Metal Detector Servo (WO1)
                        TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    
    // Initial Pulse Widths
    TCA0.SINGLE.CMP0 = 1500;  // Steering Servo
    TCA0.SINGLE.CMP1 = 1500;  // Metal Detector Servo
    
    // Start TCA0 with DIV8 prescaler
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
}