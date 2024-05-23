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
#define M1_PWM PB2
#define M1_PSTV_DIRECTION PB5
#define M1_NGTV_DIRECTION PB4

#define THERMISTOR PD6
#define POTENTIOMETER PD7
#define LED PB5
#define INPUT PC0

enum ROTARY_STATE {
  START,
  INTENT_CLOCKWISE,
  INTENT_ANTICLOCKWISE,
  CLOCKWISE,
  ANTICLOCKWISE
};
volatile float motor_duty = 0.7;
volatile bool rotary_button_toggle = 0;
volatile int rotary_state = START;


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
          rotary_state = START;
        }
      }
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

  //Initialise PWM, phase correct to have TOP = OCR1A
  bitSet(TCCR1B, WGM13);
  bitSet(TCCR1A, WGM10);
  bitSet(TCCR1A, WGM11);

  bitSet(TCCR1A, COM1B1);

  //Initialise TC1 with prescaler of 8
  bitSet(TCCR1B, CS11);

  //Set PWM pin, Positive & Negative direction pins
  bitSet(DDRB, M1_PWM);
  bitSet(DDRB, M1_PSTV_DIRECTION);
  bitSet(PORTB, M1_PSTV_DIRECTION);
  bitSet(DDRB, M1_NGTV_DIRECTION);
  bitClear(PORTB, M1_NGTV_DIRECTION);

  OCR1A = 10000;
  OCR1B = (int)(10000 * motor_duty);
  sei();

  while(1)
  {
    bitSet(PORTB,PD6);
    bitClear(PORTB,PD6);

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

    usart_tx_string(">DT:");
    usart_tx_float(bitRead(PIND, ROTARY_DT),7,1);
    usart_transmit('\n');

    usart_tx_string(">CLK:");
    usart_tx_float(bitRead(PIND, ROTARY_CLK),7,1);
    usart_transmit('\n');

    usart_tx_string(">STATE:");
    usart_tx_float(rotary_state,7,1);
    usart_transmit('\n');

    handle_rotary();

    if(COMPARE && !rotary_button_toggle)
    {
        bitSet(PORTB,M1_PSTV_DIRECTION);
    }
    else
    {
        bitClear(PORTB,M1_PSTV_DIRECTION);
    }
    
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
