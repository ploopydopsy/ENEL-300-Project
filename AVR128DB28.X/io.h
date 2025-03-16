#ifndef AVR_IO_H
#define AVR_IO_H

#include "../stdint.h"  // relative path to your stdint.h if needed

// Helper macros
#define _BV(bit) (1 << (bit))

// Define bit masks
#define PIN0_bm _BV(0)
#define PIN1_bm _BV(1)
#define PIN2_bm _BV(2)
#define PIN3_bm _BV(3)
#define PIN4_bm _BV(4)
#define PIN5_bm _BV(5)
#define PIN6_bm _BV(6)
#define PIN7_bm _BV(7)

// Minimal PORT structure
typedef struct {
    volatile unsigned char DIRSET; // Set direction bits
    volatile unsigned char DIRCLR; // Clear direction bits
    volatile unsigned char OUTSET; // Set output bits
    volatile unsigned char OUTCLR; // Clear output bits
    volatile unsigned char OUTTGL; // Toggle output bits
    volatile unsigned char IN;     // Read input bits
} PORT_t;

// We'll simulate two ports (A and D)
extern PORT_t PORTA;
extern PORT_t PORTD;

// Minimal TWI structure
typedef struct {
    volatile unsigned char MBAUD;
    volatile unsigned char MCTRLA;
    volatile unsigned char MCTRLB;
    volatile unsigned char MDATA;
    volatile unsigned char MSTATUS;
} TWI_t;

// We'll simulate TWI0
extern TWI_t TWI0;

// Some TWI bit masks
#define TWI_ENABLE_bm        _BV(7)
#define TWI_MCMD_REPSTART_gc _BV(6)
#define TWI_MCMD_STOP_gc     _BV(5)
#define TWI_WIF_bm           _BV(7)
#define TWI_RIF_bm           _BV(7)
#define TWI_ACKACT_ACK_gc    (0 << 4)
#define TWI_ACKACT_NACK_gc   _BV(4)

#endif // AVR_IO_H
