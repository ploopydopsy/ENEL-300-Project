/**
 * Simple NRF24L01+ Transmitter Test
 * For AVR128DB28 microcontroller
 * Sends a fixed pattern to verify communication
 */

#define F_CPU 20000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// NRF24L01+ Pins
#define CE_PIN      7  // PA7
#define CSN_PIN     2  // PC2
#define SCK_PIN     0  // PC0
#define MOSI_PIN    1  // PC1 
#define MISO_PIN    3  // PC3

// NRF24L01+ Register Addresses
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define TX_ADDR     0x10
#define RX_ADDR_P0  0x0A
#define FIFO_STATUS 0x17

// NRF24L01+ Commands
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define NOP           0xFF

// Function prototypes
void spi_init(void);
uint8_t spi_transfer(uint8_t data);
void nrf24_ce_low(void);
void nrf24_ce_high(void);
void nrf24_csn_low(void);
void nrf24_csn_high(void);
void nrf24_write_reg(uint8_t reg, uint8_t value);
uint8_t nrf24_read_reg(uint8_t reg);
void nrf24_write_reg_multi(uint8_t reg, uint8_t *data, uint8_t len);
void nrf24_init(void);
void nrf24_send(uint8_t *data);

int main(void) {
    // Clock setup
    CCP = 0xD8;  // Unlock protected registers
    CLKCTRL.OSCHFCTRLA = 0b00010100;  // Set internal oscillator to 20 MHz
    while (CLKCTRL.MCLKSTATUS & 0x01) { ; }  // Wait for clock to stabilize

    // GPIO setup
    PORTA.DIRSET = (1 << CE_PIN);           // CE as output
    PORTC.DIRSET = (1 << CSN_PIN) | (1 << SCK_PIN) | (1 << MOSI_PIN);  // CSN, SCK, MOSI as outputs
    PORTC.DIRCLR = (1 << MISO_PIN);         // MISO as input
    PORTA.OUTCLR = (1 << CE_PIN);           // CE low initially
    PORTC.OUTSET = (1 << CSN_PIN);          // CSN high initially
    
    // Initialize SPI and NRF24L01+
    spi_init();
    _delay_ms(100);  // Startup delay
    nrf24_init();
    
    // Basic test data - fixed pattern: 1234
    uint8_t data[5] = {0x00, 0x00, 0x00, 0x04, 0xD2};
    
    while (1) {
        // Send fixed test data
        nrf24_send(data);
        
        // Flash LED on PD5 for visual feedback
        PORTD.DIRSET = (1 << 5);
        PORTD.OUTSET = (1 << 5);
        _delay_ms(10);
        PORTD.OUTCLR = (1 << 5);
        
        // Check status - retry if failed
        uint8_t status = nrf24_read_reg(STATUS);
        if (status & (1 << 4)) {  // MAX_RT (maximum retries)
            // Clear MAX_RT flag
            nrf24_write_reg(STATUS, (1 << 4));
            // Flush TX FIFO
            nrf24_csn_low();
            spi_transfer(FLUSH_TX);
            nrf24_csn_high();
        }
        
        _delay_ms(1000);  // Send once per second
    }
    
    return 0;
}

void spi_init(void) {
    // Configure SPI master mode
    SPI0.CTRLA = SPI_MASTER_bm | SPI_ENABLE_bm;
    SPI0.CTRLB = SPI_SSD_bm;  // Disable SS pin functionality
}

uint8_t spi_transfer(uint8_t data) {
    SPI0.DATA = data;
    while(!(SPI0.INTFLAGS & SPI_IF_bm));  // Wait for transfer to complete
    return SPI0.DATA;
}

void nrf24_ce_low(void) {
    PORTA.OUTCLR = (1 << CE_PIN);
}

void nrf24_ce_high(void) {
    PORTA.OUTSET = (1 << CE_PIN);
}

void nrf24_csn_low(void) {
    PORTC.OUTCLR = (1 << CSN_PIN);
}

void nrf24_csn_high(void) {
    PORTC.OUTSET = (1 << CSN_PIN);
}

void nrf24_write_reg(uint8_t reg, uint8_t value) {
    nrf24_csn_low();
    spi_transfer(W_REGISTER | reg);
    spi_transfer(value);
    nrf24_csn_high();
}

uint8_t nrf24_read_reg(uint8_t reg) {
    nrf24_csn_low();
    spi_transfer(R_REGISTER | reg);
    uint8_t value = spi_transfer(NOP);
    nrf24_csn_high();
    return value;
}

void nrf24_write_reg_multi(uint8_t reg, uint8_t *data, uint8_t len) {
    nrf24_csn_low();
    spi_transfer(W_REGISTER | reg);
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer(data[i]);
    }
    nrf24_csn_high();
}

void nrf24_init(void) {
    _delay_ms(100);  // Power-on reset delay
    
    // Basic configuration for reliable test
    nrf24_write_reg(CONFIG, 0x0E);      // PTX, 2-byte CRC, Power up
    nrf24_write_reg(EN_AA, 0x01);       // Auto-ACK on pipe 0
    nrf24_write_reg(EN_RXADDR, 0x01);   // Enable pipe 0
    nrf24_write_reg(SETUP_AW, 0x03);    // 5-byte address
    nrf24_write_reg(SETUP_RETR, 0x2F);  // 15 retries, 750us delay
    nrf24_write_reg(RF_CH, 76);         // Channel 76
    nrf24_write_reg(RF_SETUP, 0x06);    // 1Mbps, 0dBm
    
    // Set TX address (5 bytes)
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nrf24_write_reg_multi(TX_ADDR, addr, 5);
    nrf24_write_reg_multi(RX_ADDR_P0, addr, 5);  // For ACK
    
    // Clear status
    nrf24_write_reg(STATUS, 0x70);
    
    // Flush TX FIFO
    nrf24_csn_low();
    spi_transfer(FLUSH_TX);
    nrf24_csn_high();
    
    _delay_ms(2);  // Power-up delay
}

void nrf24_send(uint8_t *data) {
    // Put CE low
    nrf24_ce_low();
    
    // Write payload (5 bytes)
    nrf24_csn_low();
    spi_transfer(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < 5; i++) {
        spi_transfer(data[i]);
    }
    nrf24_csn_high();
    
    // Pulse CE for >10us to start transmission
    nrf24_ce_high();
    _delay_us(15);
    nrf24_ce_low();
}