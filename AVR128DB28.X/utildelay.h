#ifndef UTIL_DELAY_H
#define UTIL_DELAY_H

#ifndef F_CPU
#define F_CPU 4000000UL
#endif

// _delay_us and _delay_ms functions implemented as simple busy-wait loops.
// These are not very accurate and are intended only for simple applications.

static inline void _delay_us(double __us) {
    volatile uint32_t count = (uint32_t)((F_CPU / 1000000UL) * __us);
    while(count--) {
        __asm__ __volatile__("nop");
    }
}

static inline void _delay_ms(double __ms) {
    while (__ms--) {
        _delay_us(1000);
    }
}

#endif // UTIL_DELAY_H
