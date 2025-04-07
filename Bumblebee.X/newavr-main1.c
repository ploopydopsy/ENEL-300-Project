#define F_CPU 20000000UL  // Intended 20 MHz

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// -----------------------
// TM1637 Display Settings
// -----------------------
#define TM1637_CLK_PIN 2  // PA2
#define TM1637_DIO_PIN 3  // PA3

#define TM1637_BRIGHTNESS 0x0F
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88

// -----------------------
// RX480E Receiver Module Settings
// -----------------------
#define RX_SIGNAL_PIN 2  // PD2 - Connected to D0 of RX module

// -----------------------
// Function Prototypes
// -----------------------
void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t b);
void tm1637_init(void);
void tm1637_display(uint8_t segments[]);
void display_value(uint16_t value);
void display_raw_binary(uint8_t value);

// -----------------------
// Main
// -----------------------
int main(void) {
    // --- CLOCK CONFIGURATION (Intended 20 MHz) ---
    CCP = 0xD8;  // Unlock protected registers
    CLKCTRL.OSCHFCTRLA = 0b00010100;  // Set internal oscillator to a 20 MHz
    while (CLKCTRL.MCLKSTATUS & 0x01) { ; }  // Wait for clock to stabilize

    // --- TM1637 Display INIT (using PA2 & PA3) ---
    PORTA.DIRSET |= (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    PORTA.OUTSET |= (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    tm1637_init();

    // --- Configure PD2 as input for receiver ---
    PORTD.DIRCLR = (1 << RX_SIGNAL_PIN);  // PD2 as input
    PORTD.PIN2CTRL = 0x00;                // No pull-up
    
    _delay_ms(500);  // Allow peripherals to settle
    display_value(0);

    // Counter for toggle detection
    uint8_t toggle_count = 0;
    uint8_t last_state = 0;
    
    // Main loop: Read input state and display it
    while (1) {
        // Read direct state of the pin
        uint8_t current_state = (PORTD.IN & (1 << RX_SIGNAL_PIN)) ? 1 : 0;
        
        // Count state changes (for debugging)
        if (current_state != last_state) {
            toggle_count++;
            last_state = current_state;
        }
        
        // Show the raw input state (0 or 1) and toggle count
        if (current_state) {
            // Value when signal is HIGH
            display_raw_binary(1);
        } else {
            // Value when signal is LOW
            display_raw_binary(0);
        }
        
        _delay_ms(100);  // Update display at 10Hz
    }
    
    return 0;
}

// -----------------------
// TM1637 Display Functions
// -----------------------
void tm1637_init(void) {
    _delay_ms(50);
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_DATA);
    tm1637_stop();
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_DISPLAY_CTRL | (TM1637_BRIGHTNESS & 0x07));
    tm1637_stop();
    uint8_t clear_segments[4] = {0, 0, 0, 0};
    tm1637_display(clear_segments);
}

// Display for raw binary signal (0 or 1)
void display_raw_binary(uint8_t value) {
    uint8_t segments[4];
    
    // Display the binary value (0 or 1)
    if (value) {
        segments[0] = 0x06;  // 1
    } else {
        segments[0] = 0x3F;  // 0
    }
    
    // Other three positions show text "bin"
    segments[1] = 0x7C;  // b
    segments[2] = 0x30;  // i
    segments[3] = 0x54;  // n
    
    tm1637_display(segments);
}

void display_value(uint16_t value) {
    uint8_t segments[4];
    
    // First digit (hundreds)
    if (value >= 100) {
        uint8_t digit = (value / 100) % 10;
        switch(digit) {
            case 0: segments[0] = 0x3F; break; // 0
            case 1: segments[0] = 0x06; break; // 1
            case 2: segments[0] = 0x5B; break; // 2
            case 3: segments[0] = 0x4F; break; // 3
            case 4: segments[0] = 0x66; break; // 4
            case 5: segments[0] = 0x6D; break; // 5
            case 6: segments[0] = 0x7D; break; // 6
            case 7: segments[0] = 0x07; break; // 7
            case 8: segments[0] = 0x7F; break; // 8
            case 9: segments[0] = 0x6F; break; // 9
            default: segments[0] = 0x00;
        }
    } else {
        segments[0] = 0;
    }
    
    // Second digit (tens)
    if (value >= 10) {
        uint8_t digit = (value / 10) % 10;
        switch(digit) {
            case 0: segments[1] = 0x3F; break; // 0
            case 1: segments[1] = 0x06; break; // 1
            case 2: segments[1] = 0x5B; break; // 2
            case 3: segments[1] = 0x4F; break; // 3
            case 4: segments[1] = 0x66; break; // 4
            case 5: segments[1] = 0x6D; break; // 5
            case 6: segments[1] = 0x7D; break; // 6
            case 7: segments[1] = 0x07; break; // 7
            case 8: segments[1] = 0x7F; break; // 8
            case 9: segments[1] = 0x6F; break; // 9
            default: segments[1] = 0x00;
        }
    } else {
        segments[1] = 0;
    }
    
    // Third digit (ones)
    uint8_t ones = value % 10;
    switch(ones) {
        case 0: segments[2] = 0x3F; break; // 0
        case 1: segments[2] = 0x06; break; // 1
        case 2: segments[2] = 0x5B; break; // 2
        case 3: segments[2] = 0x4F; break; // 3
        case 4: segments[2] = 0x66; break; // 4
        case 5: segments[2] = 0x6D; break; // 5
        case 6: segments[2] = 0x7D; break; // 6
        case 7: segments[2] = 0x07; break; // 7
        case 8: segments[2] = 0x7F; break; // 8
        case 9: segments[2] = 0x6F; break; // 9
        default: segments[2] = 0x00;
    }
    
    // Fourth digit - could show a custom indicator
    segments[3] = 0x00;  // Blank
    
    tm1637_display(segments);
}

void tm1637_display(uint8_t segments[]) {
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_ADDR);
    for (uint8_t i = 0; i < 4; i++) {
        tm1637_write_byte(segments[i]);
    }
    tm1637_stop();
}

void tm1637_start(void) {
    PORTA.OUTSET = (1 << TM1637_DIO_PIN);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
}

void tm1637_stop(void) {
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}

void tm1637_write_byte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
        PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        if (b & 0x01)
            PORTA.OUTSET = (1 << TM1637_DIO_PIN);
        else
            PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
        _delay_us(2);
        PORTA.OUTSET = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        b >>= 1;
    }
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTA.DIRCLR = (1 << TM1637_DIO_PIN);
    _delay_us(5);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTA.DIRSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}