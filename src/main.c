#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "usart.h"
#include "bit.h"

#define ROTARY_CLK PD4
#define ROTARY_DT PD3
#define ROTARY_BTN PD2 // normally open, shorted to GND on press

#define M1_PWM PD5
#define M1_PSTV_DIRECTION PB5
#define M1_NGTV_DIRECTION PB4

#define THERMISTOR PD6
#define POTENTIOMETER PD7
#define INPUT PC0

volatile float motor_duty = 0.5;
//prevents external interrupt from triggering too many increments
volatile bool debounce = 0;
volatile bool rotary_button_toggle = 0;

void adc_init();
float result_to_tempC(uint16_t result);

ISR(INT1_vect) {
  if (debounce) return;
  debounce = 1;
  _delay_ms(5);

  if (bitRead(PIND, ROTARY_CLK) == bitRead(PIND, ROTARY_DT)) {
    // clockwise
    motor_duty += 0.05;
  } else {
    // anti clock
    motor_duty -= 0.05;
  }

  if (motor_duty > 0.98) {
    motor_duty = 0.98;
  }
  if (motor_duty < 0.5) {
    motor_duty = 0.5;
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
  adc_init();

  // Set as inputs
  bitClear(DDRD,THERMISTOR);    // AIN0
  bitClear(DDRD,POTENTIOMETER); // AIN1


  int channelswap = 0;
  int COMPARE;
  uint16_t therm_read, pot_read;

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

  //Initialise PWM, phase correct to have TOP = OCRA
  bitSet(TCCR0B, WGM02);
  bitSet(TCCR0A, WGM00);
  bitSet(TCCR0A, COM0B1);

  //Initialise TC1 with prescaler of 8
  bitSet(TCCR0B, CS01);

  bitClear(TCCR0B, FOC0A);
  bitClear(TCCR0B, FOC0B);

  //Set PWM pin, Positive & Negative direction pins
  bitSet(DDRD, M1_PWM);
  bitSet(DDRB, M1_PSTV_DIRECTION);
  bitSet(PORTB, M1_PSTV_DIRECTION);
  bitSet(DDRB, M1_NGTV_DIRECTION);
  bitClear(PORTB, M1_NGTV_DIRECTION);

  OCR0A = 255;
  sei();

  while(1)
  {
    debounce = 0;
    OCR0B = 255 * motor_duty;
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

    bitSet(ADCSRA,ADSC); // Start ADC conversion
    while(bitRead(ADCSRA,ADSC));
    // Read result
    if(!channelswap)
    {
        therm_read = ADC;

        usart_tx_string(">Thermistor Analog IN:");
        usart_tx_float(therm_read,7,1);
        usart_transmit('\n');

        usart_tx_string(">Thermistor Temperature (C):");
        usart_tx_float(result_to_tempC(therm_read),7,1);
        usart_transmit('\n');
    }
    if(channelswap)
    {
        pot_read = ADC;
        usart_tx_string(">Potentiometer Analog IN:");
        usart_tx_float(pot_read,7,1);
        usart_transmit('\n');

        usart_tx_string(">Potentiometer Temperature (C):");
        usart_tx_float(result_to_tempC(pot_read),7,1);
        usart_transmit('\n');
    }

    channelswap = !channelswap;
    ADMUX &= 0xF0;
    ADMUX |= channelswap;

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
