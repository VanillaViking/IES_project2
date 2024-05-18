#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "usart.h"
#include "bit.h"

volatile bool rotary_pin_a = 0;
volatile bool rotary_pin_b = 0;
volatile bool debounce = 0;

ISR(INT1_vect) {
  debounce = 1;

  for (int i = 0; i < 10; i++) {
    rotary_pin_a = bitRead(PIND, PD2);
    rotary_pin_b = bitRead(PIND, PD3);
  }
}


int main(void) {
  // beginning of pain
  cli();
  usart_init(8);

  bitSet(DDRB,PB5);

  bitClear(DDRD,PD2);
  bitClear(DDRD,PD3);
  
  /* bitSet(EICRA, ISC01); */
  /* bitClear(EICRA, ISC00); */
  /* bitSet(EIMSK, INT0); */

  bitSet(EICRA, ISC11);
  bitClear(EICRA, ISC10);
  bitSet(EIMSK, INT1);

  sei();
  
  while(1)
  {
    bitSet(PORTB,PB5);
    _delay_ms(100);
    bitClear(PORTB,PB5);
    _delay_ms(100);
    debounce = 0;
  
    usart_tx_float(rotary_pin_a, 1, 1);
    usart_transmit(',');
    usart_tx_float(rotary_pin_b, 1, 1);
    usart_transmit('\n');
  }
}
