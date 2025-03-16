#ifndef UTIL_DELAY_H
#define UTIL_DELAY_H

#ifndef F_CPU
#define F_CPU 4000000UL  // 4 MHz default
#endif

// Very rough busy-wait loops
static inline void _delay_us(double __us) {
    volatile unsigned long count = (unsigned long)((F_CPU / 1000000UL) * __us);
    while(count--) {
        __asm__ __volatile__("nop");
    }
}

static inline void _delay_ms(double __ms) {
    while(__ms--) {
        _delay_us(1000);
    }
}

#endif // UTIL_DELAY_H
