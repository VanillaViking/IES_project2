#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "usart.h"
#include "bit.h"

#define ROTARY_CLK PD7
#define ROTARY_DT PD3
// normally open, shorted to GND on press
#define ROTARY_BTN PD2

volatile float motor_duty = 0.5;
//prevents external interrupt from triggering too many increments
volatile bool debounce = 0;
volatile bool rotary_button_toggle = 0;

ISR(INT1_vect) {
  if (debounce) return;
  debounce = 1;
  _delay_ms(10);

  if (bitRead(PIND, ROTARY_CLK) == bitRead(PIND, ROTARY_DT)) {
    // clockwise
    motor_duty += 0.05;
  } else {
    // anti clock
    motor_duty -= 0.05;
  }

  if (motor_duty > 0.9) {
    motor_duty = 0.9;
  }
  if (motor_duty < 0) {
    motor_duty = 0;
  }

}

// rotary button
ISR(INT0_vect) {
  _delay_ms(10);
  if (bitRead(PIND, ROTARY_BTN) == 0) {
    rotary_button_toggle = !rotary_button_toggle;
  }
}


int main(void) {
  // beginning of pain
  cli();
  usart_init(8);

  bitSet(DDRB,PB5);

  bitClear(DDRD,ROTARY_CLK);
  bitClear(DDRD,ROTARY_DT);
  bitClear(DDRD,ROTARY_BTN);
  
  // enable pullup resistor
  bitSet(PORTD, ROTARY_BTN);
  
  bitSet(EICRA, ISC01);
  bitClear(EICRA, ISC00);
  bitSet(EIMSK, INT0);

  bitClear(EICRA, ISC11);
  bitSet(EICRA, ISC10);
  bitSet(EIMSK, INT1);

  sei();

  while(1)
  {
    bitSet(PORTB,PB5);
    _delay_ms(100);
    bitClear(PORTB,PB5);
    _delay_ms(100);
    debounce = 0;

    usart_tx_string(">a:");
    usart_tx_float(motor_duty, 1, 2);
    usart_tx_string(">b:");
    usart_tx_float(rotary_button_toggle, 1, 2);
    usart_transmit('\n');
  }
}
