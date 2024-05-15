#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"

int main(void) {
  // beginning of pain
  bitSet(DDRB,PB5);
  
  while(1)
  {
    bitSet(PORTB,PB5);
    _delay_ms(500);
    bitClear(PORTB,PB5);
    _delay_ms(500);
  }
}
