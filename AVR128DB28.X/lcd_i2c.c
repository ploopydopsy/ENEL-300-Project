#include "lcd_i2c.h"
#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 8000000UL
#define F_SCL 100000UL
#define TWBR_val (((F_CPU / F_SCL) - 16) / 2)

#define LCD_I2C_ADDR 0x27
#define LCD_BACKLIGHT 0x08
#define LCD_EN 0x04
#define LCD_RS 0x01

void i2c_init(void) {
    TWSR = 0;
    TWBR = TWBR_val;
}

uint8_t i2c_start(uint8_t address) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != 0x08) return 1;
    
    TWDR = address;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    return (TWSR & 0xF8) != 0x18;
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    _delay_us(10);
}

void lcd_write_byte(uint8_t data) {
    i2c_start(LCD_I2C_ADDR << 1);
    i2c_write(data | LCD_BACKLIGHT);
    i2c_stop();
}

void lcd_command(uint8_t cmd) {
    lcd_write_byte((cmd & 0xF0) | LCD_EN);
    lcd_write_byte((cmd << 4) | LCD_EN);
    _delay_ms(2);
}

void lcd_data(uint8_t data) {
    lcd_write_byte((data & 0xF0) | LCD_EN | LCD_RS);
    lcd_write_byte((data << 4) | LCD_EN | LCD_RS);
    _delay_ms(2);
}

void lcd_print(const char *str) {
    while (*str) lcd_data(*str++);
}

void lcd_init(void) {
    i2c_init();
    _delay_ms(50);
    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_command(0x01);
    _delay_ms(2);
}
