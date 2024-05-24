#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "usart.h"
#include "bit.h"

#define ROTARY_CLK PD2
#define ROTARY_DT PD3
// normally open, shorted to GND on press
#define ROTARY_BTN PD4

#define M1_PWM PD5
#define M1_PSTV_DIRECTION PB5
#define M1_NGTV_DIRECTION PB4

#define THERMISTOR PD6
#define POTENTIOMETER PD7
#define LED PB5
#define INPUT PC0

#define BUFFER_SIZE 50

enum ROTARY_STATE {
  START,
  INTENT_CLOCKWISE,
  INTENT_ANTICLOCKWISE,
  CLOCKWISE,
  ANTICLOCKWISE
};

volatile float motor_duty = 0.7;
volatile bool rotary_button_toggle = 1;
volatile int rotary_state = START;

unsigned char data_buffer[BUFFER_SIZE];
unsigned char *pTx = data_buffer;

void adc_init();
float result_to_tempC(uint16_t result);
void handle_rotary();

// DT INTERRUPT
ISR(INT1_vect) {
  bool dt = bitRead(PIND, ROTARY_DT);
  bool clk = bitRead(PIND, ROTARY_CLK);

  switch (rotary_state) {
    case START:
      if (dt) {
        rotary_state = INTENT_ANTICLOCKWISE;
      } else {
        if (clk) {
          rotary_state = INTENT_ANTICLOCKWISE;
        } else {
          // invalid state
          rotary_state = START;
        }
      }
      break;

    case INTENT_CLOCKWISE:
      if (dt) {
        rotary_state = CLOCKWISE;
      } else {
        if (!clk) {
          rotary_state = CLOCKWISE;
        } else {
          // invalid state
          rotary_state = START;
        }
      }
  }
  
}

// CLK INTERRUPT
ISR(INT0_vect) {
  bool dt = bitRead(PIND, ROTARY_DT);
  bool clk = bitRead(PIND, ROTARY_CLK);

  switch (rotary_state) {
    case START:
      if (clk) {
        rotary_state = INTENT_CLOCKWISE;
      } else {
        if (dt) {
          rotary_state = INTENT_CLOCKWISE;
        } else {
          // invalid state
          rotary_state = START;
        }
      }
      break;

    case INTENT_ANTICLOCKWISE:
      if (clk) {
        rotary_state = ANTICLOCKWISE;
      } else {
        if (!dt) {
          rotary_state = ANTICLOCKWISE;
        } else {
          // invalid state
          rotary_state = START;
        }
      }
  }
     
}

ISR(USART_UDRE_vect) {
  if(*pTx == '\0') {
    pTx = data_buffer;
  } else {
    UDR0 = *pTx;
    pTx++;
  }
}

int main(void) {
  // beginning of pain
  cli();
  usart_init(8);
  adc_init();

  bitSet(DDRB,PB5);

  // Set as inputs
  bitClear(DDRD,THERMISTOR);    // AIN0
  bitClear(DDRD,POTENTIOMETER); // AIN1
  
  bitSet(DDRB,LED); // LED

  int channelswap = 0;
  int COMPARE;
  uint16_t therm_read, pot_read;

  bitClear(DDRD,ROTARY_CLK);
  bitClear(DDRD,ROTARY_DT);
  bitClear(DDRD,ROTARY_BTN);

  // enable pullup resistor
  bitSet(PORTD, ROTARY_BTN);

  bitClear(EICRA, ISC01);
  bitSet(EICRA, ISC00);
  bitSet(EIMSK, INT0);

  bitClear(EICRA, ISC11);
  bitSet(EICRA, ISC10);
  bitSet(EIMSK, INT1);

  //Initialise PWM, phase correct to have TOP = OCRA
  bitSet(TCCR0B, WGM02);
  bitSet(TCCR0A, WGM00);
  bitSet(TCCR0A, COM0B1);

  //Initialise TC0 with prescaler of 1024
  bitSet(TCCR0B, CS02);
  bitSet(TCCR0B, CS00);

  bitClear(TCCR0B, FOC0A);
  bitClear(TCCR0B, FOC0B);

  //Set PWM pin, Positive & Negative direction pins
  bitSet(DDRD, M1_PWM);
  bitSet(DDRB, M1_PSTV_DIRECTION);
  bitSet(PORTB, M1_PSTV_DIRECTION);
  bitSet(DDRB, M1_NGTV_DIRECTION);
  bitClear(PORTB, M1_NGTV_DIRECTION);

  //set USART interrupts
  bitSet(UCSR0B, UDRIE0);
  bitSet(SREG, SREG_I);

  OCR0A = 78;
  sei();

  while(1)
  {
    bitSet(PORTB,PD6);
    bitClear(PORTB,PD6);

    if(COMPARE > 0 && rotary_button_toggle) {
      OCR0B = (int)(78 * motor_duty);
    }
    else {
      OCR0B = 1;
    }

    usart_tx_string(">a:");
    usart_tx_float(motor_duty, 1, 2);
    usart_transmit('\n');
    usart_tx_string(">b:");
    usart_tx_float(rotary_button_toggle, 1, 2);
    usart_transmit('\n');
    COMPARE = 35*bitRead(ACSR,ACO);

    usart_tx_string(">OUTPUT:");
    usart_tx_float(COMPARE,3,1);
    usart_transmit('\n');

    handle_rotary();

    
    bitSet(ADCSRA,ADSC); // Start ADC conversion
    while(bitRead(ADCSRA,ADSC));
    // Read result
    if(!channelswap)
    {
        therm_read = ADC;

        usart_tx_string(">Thermistor Temperature (C):");
        usart_tx_float(result_to_tempC(therm_read),7,1);
        usart_transmit('\n');
    }
    if(channelswap)
    {
        pot_read = ADC;

        usart_tx_string(">Potentiometer Temperature (C):");
        usart_tx_float(result_to_tempC(pot_read),7,1);
        usart_transmit('\n');
    }

    channelswap = !channelswap;
    ADMUX &= 0xF0;
    ADMUX |= channelswap;

  }
}

void handle_rotary() {
  if (rotary_state == CLOCKWISE) {
    motor_duty += 0.05;
    rotary_state = START;
  } else if (rotary_state == ANTICLOCKWISE) {
    motor_duty -= 0.05;
    rotary_state = START;
  }

  if (bitRead(PIND, ROTARY_DT) == bitRead(PIND, ROTARY_CLK)) {
    rotary_state = START;
  }
  if (motor_duty > 0.95) {
    motor_duty = 0.95;
  }
  if (motor_duty < 0.1) {
    motor_duty = 0.1;
  }
  if (bitRead(PIND, ROTARY_BTN) == 0) {
    _delay_ms(60);
    if (bitRead(PIND, ROTARY_BTN) == 0) {
      rotary_button_toggle = !rotary_button_toggle;
    }
  }
}

float result_to_tempC(uint16_t result)
{
    float tempReading = result;
    double tempK = log(10000.0 * ((1023.0 / tempReading - 1)));
    tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK)) * tempK); // Kelvin
    float tempC = tempK - 273.15;
    return tempC;
}
void adc_init()
{
    // Configure ADC reference Voltage - 5V
    bitSet(ADMUX, REFS0);
    bitClear(ADMUX, REFS1);

    // Configure ADC clock prescaler - P = 128 (111)
    bitSet(ADCSRA, ADPS0);
    bitSet(ADCSRA, ADPS1);
    bitSet(ADCSRA, ADPS2);  

    // Configure ADC Input channel - ADC0 (0000) - PC0/A0
    bitClear(DDRC, INPUT); // PC0 as INPUT - Analog IN
    int channel = 0;
    ADMUX &= 0xF0;    // Clear bits 3:0
    ADMUX |= channel; // Set bits 3:0 = channel

    // Mode Selection - Single Conversion Mode
    bitClear(ADCSRA, ADATE);

    // Enable ADC
    bitSet(ADCSRA, ADEN);
}
