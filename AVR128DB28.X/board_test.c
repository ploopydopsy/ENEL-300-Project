/*
 * File:   board test.c
 * Author: williamadebiyi
 *
 * Created on January 26, 2025, 1:39 PM
 */


#include <avr/io.h>

#define LONG_DELAY_VALUE 5000000
#define SHORT_DELAY_VALUE 500000

int main(void) {
  
  // Enable Port A and D output pins.
  PORTA.DIRSET = 0b11111111;
  PORTD.DIRSET = 0b11111110;
  
  // Set output pins low.
  PORTA.OUT = 0b00000000;  
  PORTD.OUT = 0b00000000;
  
  // Pause for a few seconds.
  // NOTE: Later in ENCM369, you'll learn how to use timers for delays.
  //       In general, using nothing loops like this is bad practice.
  
  unsigned long counter=0;
  while(counter < LONG_DELAY_VALUE){
    counter++;
  }
  
  
  
  
  /* Replace with your application code */
  while (1) {
    
    // Set output pins high.
    PORTA.OUT = 0b11111111;  
    PORTD.OUT = 0b11111110;

    // Delay
    counter=0;
    while(counter < SHORT_DELAY_VALUE){
      counter++;
    }
    
    // Set output pins low.
    PORTA.OUT = 0b00000000;  
    PORTD.OUT = 0b00000000;
    
    // Delay
    counter=0;
    while(counter < SHORT_DELAY_VALUE){
      counter++;
    }
    
  }
}
