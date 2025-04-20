#define F_CPU 4000000UL
// NEVER TESTED DONT KNOW IF IT DOES ANYTHIGN OR DOES WHAY I WANT BUT I TOLD CHAT TO MAKE IT 
//rgb_headlight_listener.c
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

// === Pin Mapping ===
#define RGB_PIN_R 6     // PA6
#define RGB_PIN_G 7     // PA7
#define RGB_PIN_B 4     // PA4
#define HEADLIGHT_OUT 5 // PA5

#define RGB_INPUT_PIN 7 // PD7 (from ESP32 GPIO2)
#define HEADLIGHT_INPUT_PIN 5 // PD5 (from ESP32 GPIO27)

// === Color Presets ===
const bool colorLUT[8][3] = {
    {false, false, false}, // 0 = OFF
    {true,  false, false}, // 1 = RED
    {false, true,  false}, // 2 = GREEN
    {false, false, true }, // 3 = BLUE
    {true,  true,  false}, // 4 = YELLOW
    {true,  false, true }, // 5 = MAGENTA
    {false, true,  true }, // 6 = CYAN
    {true,  true,  true }  // 7 = WHITE
};

// === Function Prototypes ===
void init_rgb_and_headlight_io(void);
void set_rgb(bool r, bool g, bool b);
void set_headlight(bool on);
uint8_t listen_for_rgb_command(void);

int main(void) {
    init_rgb_and_headlight_io();

    while (1) {
        // Update headlight based on PD5 input
        bool headlight_state = (PORTD.IN & (1 << HEADLIGHT_INPUT_PIN));
        set_headlight(headlight_state);

        // Listen for RGB color signal from PD7
        uint8_t color_index = listen_for_rgb_command();
        if (color_index <= 7) {
            set_rgb(
                colorLUT[color_index][0],
                colorLUT[color_index][1],
                colorLUT[color_index][2]
            );
        }

        _delay_ms(50); // debounce/signal spacing
    }
}

void init_rgb_and_headlight_io(void) {
    // RGB and headlight output pins
    PORTA.DIRSET = (1 << RGB_PIN_R) | (1 << RGB_PIN_G) | (1 << RGB_PIN_B) | (1 << HEADLIGHT_OUT);
    PORTA.OUTSET = (1 << RGB_PIN_R) | (1 << RGB_PIN_G) | (1 << RGB_PIN_B); // common anode = OFF
    PORTA.OUTCLR = (1 << HEADLIGHT_OUT); // start headlight OFF

    // RGB signal and headlight input pins
    PORTD.DIRCLR = (1 << RGB_INPUT_PIN) | (1 << HEADLIGHT_INPUT_PIN); // inputs
    PORTD.PIN7CTRL = PORT_PULLUPEN_bm;  // enable pullup for RGB input
    PORTD.PIN5CTRL = PORT_PULLUPEN_bm;  // enable pullup for headlight input
}

// === Common Anode RGB ===
void set_rgb(bool r, bool g, bool b) {
    if (r) PORTA.OUTCLR = (1 << RGB_PIN_R);
    else   PORTA.OUTSET = (1 << RGB_PIN_R);

    if (g) PORTA.OUTCLR = (1 << RGB_PIN_G);
    else   PORTA.OUTSET = (1 << RGB_PIN_G);

    if (b) PORTA.OUTCLR = (1 << RGB_PIN_B);
    else   PORTA.OUTSET = (1 << RGB_PIN_B);
}

// === Headlight GPIO ===
void set_headlight(bool on) {
    if (on)
        PORTA.OUTSET = (1 << HEADLIGHT_OUT); // ON
    else
        PORTA.OUTCLR = (1 << HEADLIGHT_OUT); // OFF
}

// === RGB Pulse Train Listener ===
uint8_t listen_for_rgb_command(void) {
    // Wait for initial HIGH pulse (~10ms)
    while (!(PORTD.IN & (1 << RGB_INPUT_PIN))) ;
    _delay_ms(8);
    while (PORTD.IN & (1 << RGB_INPUT_PIN)) ;
    _delay_ms(15); // LOW between pulses

    // Count pulses
    uint8_t count = 0;
    while (1) {
        uint16_t timeout = 0;
        while (!(PORTD.IN & (1 << RGB_INPUT_PIN))) {
            _delay_ms(1);
            timeout++;
            if (timeout > 100) return count;
        }
        _delay_ms(5);
        count++;
        while (PORTD.IN & (1 << RGB_INPUT_PIN)) _delay_ms(1);
        if (count > 7) break;
    }
    return count;
}
