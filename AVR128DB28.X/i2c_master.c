#include "i2c_master.h"

#ifndef F_CPU
#define F_CPU 4000000UL
#endif

#define F_SCL 100000UL // I2C clock speed
#define PRESCALER 1
#define TWBR_VALUE (((F_CPU / F_SCL) - 16) / (2 * PRESCALER))

void i2c_init(void) {
    // Pretend to configure TWI0 for 100kHz
    TWI0.MBAUD = (unsigned char)TWBR_VALUE;
    TWI0.MCTRLA = TWI_ENABLE_bm;  // Enable TWI
}

void i2c_start(void) {
    // Repeated START
    TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;
}

void i2c_stop(void) {
    // STOP condition
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}

void i2c_write(uint8_t data) {
    TWI0.MDATA = data;
    // Wait for "WIF" bit
    // (In real hardware, you'd loop until TWI0.MSTATUS & TWI_WIF_bm)
}

uint8_t i2c_read_ack(void) {
    // In real hardware, you'd set TWI0.MCTRLB = TWI_ACKACT_ACK_gc;
    // Wait for RIF
    return TWI0.MDATA;
}

uint8_t i2c_read_nack(void) {
    // In real hardware, you'd set TWI0.MCTRLB = TWI_ACKACT_NACK_gc;
    // Wait for RIF
    return TWI0.MDATA;
}
