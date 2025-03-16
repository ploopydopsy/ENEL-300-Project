/*
 * File:   level-switch.c
 * Author: gmessier
 *
 * Created on October 31, 2021, 2:00 PM
 */


#include <avr/io.h>

int main(void) {
    
    // Enable PA5 as an output pin.
    PORTA.DIRSET = 0b00100000;
    
    // Enable PA6 as an input pin.
    PORTA.DIRCLR = 0b01000000;
        
    while (1) {
        if(PORTA.IN & 0b01000000)
            PORTA.OUT &= 0b11011111;
        else
            PORTA.OUT |= 0b00100000;
    }
}
