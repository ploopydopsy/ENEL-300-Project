#include <avr/io.h>
#include <util/delay.h>
#include "TM1637.h"  // Include TM1637 library

#define F_CPU 16000000UL  // Clock speed

// ? TM1637 Display Pins
#define TM1637_CLK 2  // PA2
#define TM1637_DIO 3  // PA3

// ? HC-SR04 Sensor Pins
#define TRIG_PIN 0  // PA0
#define ECHO_PIN 1  // PA1

void init_pins() {
    PORTA.DIRSET = (1 << TRIG_PIN);  // TRIG as output
    PORTA.DIRCLR = (1 << ECHO_PIN);  // ECHO as input
}

void trigger_pulse() {
    PORTA.OUTCLR = (1 << TRIG_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << TRIG_PIN);
    _delay_us(10);
    PORTA.OUTCLR = (1 << TRIG_PIN);
}

uint16_t get_pulse_width() {
    uint16_t count = 0;

    while (!(PORTA.IN & (1 << ECHO_PIN)));  // Wait for ECHO HIGH
    while (PORTA.IN & (1 << ECHO_PIN)) {    // Count while HIGH
        _delay_us(1);
        count++;
    }

    return count;
}

uint16_t get_distance_cm() {
    trigger_pulse();
    uint16_t duration = get_pulse_width();
    return duration / 58;
}

int main(void) {
    init_pins();

    // ? Initialize TM1637 Display
    TM1637_init(TM1637_CLK, TM1637_DIO);
    TM1637_setBrightness(7);

    while (1) {
        uint16_t distance = get_distance_cm();
        
        // ? Show distance on TM1637 Display
            TM1637_displayDecimal(distance, 0);

        _delay_ms(500);
    }
}
