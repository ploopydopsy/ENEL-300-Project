#include "sensor.h"
#include <avr/io.h>
#include <util/delay.h>

#define TRIG_DDR   DDRD
#define TRIG_PORT  PORTD
#define TRIG_PIN   PD2
#define ECHO_PIN   PD3

void sensor_init(void) {
    TRIG_DDR |= (1 << TRIG_PIN);
    TRIG_PORT &= ~(1 << TRIG_PIN);
}

uint16_t measure_distance_cm(void) {
    uint32_t pulse_width = 0;
    uint16_t distance;

    TRIG_PORT &= ~(1 << TRIG_PIN);
    _delay_us(2);
    TRIG_PORT |= (1 << TRIG_PIN);
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_PIN);

    while (!(PIND & (1 << ECHO_PIN)));
    while (PIND & (1 << ECHO_PIN)) {
        _delay_us(1);
        pulse_width++;
        if (pulse_width > 30000) break;
    }

    distance = pulse_width / 58;
    return distance;
}
