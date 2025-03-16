#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include "avr/io.h"
#include "stdint.h"

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);

#endif // I2C_MASTER_H
