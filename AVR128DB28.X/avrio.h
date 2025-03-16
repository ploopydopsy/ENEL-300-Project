#ifndef AVR_IO_H
#define AVR_IO_H

#include "stdint.h"

// Helper macro to build a bit mask.
#define _BV(bit) (1 << (bit))

// Bit masks (you can add more if needed)
#define PIN0_bm _BV(0)
#define PIN1_bm _BV(1)
#define PIN2_bm _BV(2)
#define PIN3_bm _BV(3)
#define PIN4_bm _BV(4)
#define PIN5_bm _BV(5)
#define PIN6_bm _BV(6)
#define PIN7_bm _BV(7)

// Minimal structure for an 8-bit PORT
typedef struct {
    volatile uint8_t DIRSET; // Set direction bits
    volatile uint8_t DIRCLR; // Clear direction bits
    volatile uint8_t OUTSET; // Set output bits
    volatile uint8_t OUTCLR; // Clear output bits
    volatile uint8_t IN;     // Input pins read
} PORT_t;

// Declare PORTs you'll be using.
extern PORT_t PORTA;
extern PORT_t PORTD;

// Minimal structure for a TWI/I2C module registers
typedef struct {
    volatile uint8_t MBAUD;
    volatile uint8_t MCTRLA;
    volatile uint8_t MCTRLB;
    volatile uint8_t MDATA;
    volatile uint8_t MSTATUS;
} TWI_t;

// Declare the TWI module you'll be using.
extern TWI_t TWI0;

// Define some bit masks and group constants for TWI registers (values are examples)
#define TWI_ENABLE_bm        _BV(7)
#define TWI_MCMD_REPSTART_gc _BV(6)
#define TWI_MCMD_STOP_gc     _BV(5)
#define TWI_WIF_bm           _BV(7)
#define TWI_RIF_bm           _BV(7)
#define TWI_ACKACT_ACK_gc    (0 << 4)
#define TWI_ACKACT_NACK_gc   _BV(4)

#endif // AVR_IO_H
