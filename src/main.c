#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "usart.h"
#include "bit.h"
#include "usart_queue.h"

#define ROTARY_CLK PD2
#define ROTARY_DT PD3
// normally open, shorted to GND on press
#define ROTARY_BTN PD4

#define M1_PWM PD5
#define M1_PSTV_DIRECTION PB5
#define M1_NGTV_DIRECTION PB4

#define THERMISTOR PD6
#define POTENTIOMETER PD7
#define INPUT PC0

#define CHAR_BUFFER 25
#define TOP 2000
#define CLOCKF 1000 // Frequency of T1 clock

int COMPARE = 0;  // Set if Thermistor value is > Potentiometer value
uint16_t therm_read, pot_read;
char status[CHAR_BUFFER];
char system_uptime[CHAR_BUFFER];
bool risingcaptured = 0;
double numOv = 0;
double ICtimecapt;
double FanTURNONtime;
double FanTURNOFFtime;
bool status_changed = 0;
Queue usart_queue;

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

void adc_init();
void t1_init();
float result_to_tempC(uint16_t result);
void clock_to_time(double clocktime, char timedisp[]);
void handle_rotary();

ISR(TIMER1_COMPA_vect){
    numOv++;
}

/* ISR for handling AC output changes - Fan state */
ISR(ANALOG_COMP_vect){
    status_changed = 1; // Fan status has been changed as ISR has been triggered
    if(bitRead(ACSR,ACO)){ // rising edge has been detected
        strcpy(status,"FAN TURNED ON AT - ");
    }
    if(!bitRead(ACSR,ACO)){ // falling edge has been detected

        strcpy(status,"FAN TURNED OFF AT - ");
    }
    strcat(status,system_uptime); // Combine status and system uptime string for displaying
}

/* ISR for capturing time stamps of Fan state changes */
ISR(TIMER1_CAPT_vect){
    if(COMPARE == 0){ // rising edge has been captured
        FanTURNONtime = (ICR1 + numOv*TOP)/TOP;
        bitClear(TCCR1B,ICES1); // Capture on FALLING edge
    }
    if(COMPARE == 1){ // falling edge has been captured
        FanTURNOFFtime = (ICR1 + numOv*TOP)/TOP;
        bitSet(TCCR1B,ICES1); // Capture on RISING edge
    }
    _delay_ms(30);
}
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
  if (!is_empty(&usart_queue)) {
    UDR0 = dequeue(&usart_queue);
  }
}

int main(void) {
  // I/O ports
  bitClear(DDRD,THERMISTOR);    // AIN0
  bitClear(DDRD,POTENTIOMETER); // AIN1

  bitSet(ACSR,ACIE); // Analog Comparator Interrupt Enable

  char LiveFanOnTime[CHAR_BUFFER];
  char PrevFanOnTime[CHAR_BUFFER];
  char FanTurnOnTime[CHAR_BUFFER];

  int channelswap = 0;
  // Set as inputs
  bitClear(DDRD,THERMISTOR);    // AIN0
  bitClear(DDRD,POTENTIOMETER); // AIN1

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
  // bitSet(TCCR0B, CS00);

  bitClear(TCCR0B, FOC0A);
  bitClear(TCCR0B, FOC0B);

  //Set PWM pin, Positive & Negative direction pins
  bitSet(DDRD, M1_PWM);
  bitSet(DDRB, M1_PSTV_DIRECTION);
  bitSet(PORTB, M1_PSTV_DIRECTION);
  bitSet(DDRB, M1_NGTV_DIRECTION);
  bitClear(PORTB, M1_NGTV_DIRECTION);


  OCR0A = 156;
  usart_init(8); // 103-9600 bps; 8-115200
  // USART interrupt
  bitSet(UCSR0B, UDRIE0);
  bitSet(SREG, SREG_I);

  adc_init();
  t1_init();
  sei();

  usart_queue = queue_init();

  while(1)
  {
    COMPARE = bitRead(ACSR,ACO); // Determine AC output
    bitSet(PORTB,PD6);
    bitClear(PORTB,PD6);

    if(COMPARE > 0 && rotary_button_toggle) {
      OCR0B = (int)(156 * motor_duty);
      clock_to_time(numOv-FanTURNONtime,LiveFanOnTime);
      enqueue_string(&usart_queue, ">Fan Uptime:");
      enqueue_string(&usart_queue, "Fan ON duration - ");
      enqueue_string(&usart_queue, LiveFanOnTime);
      enqueue_string(&usart_queue, "|t");
      enqueue(&usart_queue, '\n');
    } else {
      OCR0B = 1;
      clock_to_time(abs(FanTURNOFFtime - FanTURNONtime),PrevFanOnTime);
      clock_to_time(FanTURNONtime,FanTurnOnTime);
      enqueue_string(&usart_queue, ">Fan Uptime:");
      enqueue_string(&usart_queue, "Fan OFF, was last turned ON at ");
      enqueue_string(&usart_queue, FanTurnOnTime);
      enqueue_string(&usart_queue, " for duration - ");
      enqueue_string(&usart_queue, PrevFanOnTime);
      enqueue_string(&usart_queue, "|t");
      enqueue(&usart_queue, '\n');
    }

    enqueue_string(&usart_queue, ">a:");
    enqueue_float(&usart_queue, motor_duty, 2, 3);
    enqueue(&usart_queue, '\n');

    enqueue_string(&usart_queue, ">OUTPUT:");
    enqueue_float(&usart_queue, COMPARE + 0.0,3,1);
    enqueue(&usart_queue, '\n');

    handle_rotary();


    bitSet(ADCSRA,ADSC); // Start ADC conversion
    while(bitRead(ADCSRA,ADSC));
    // Read result
    if(!channelswap)
    {
        therm_read = ADC;

        enqueue_string(&usart_queue, ">Thermistor Temperature (C):");
        enqueue_float(&usart_queue, result_to_tempC(therm_read),5,1);
        enqueue(&usart_queue, '\n');
    }
    if(channelswap)
    {
        pot_read = ADC;

        enqueue_string(&usart_queue, ">Potentiometer Temperature (C):");
        enqueue_float(&usart_queue, result_to_tempC(pot_read),5,1);
        enqueue(&usart_queue, '\n');
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

/* Initialises Timer1 - used for Input Capture */
void t1_init()
{
    bitSet(ACSR, ACIC);      // Enable the input capture function in TCNT1 to be triggered by AC
    bitSet(TIMSK1, ICIE1);   // TCNT1 Input Capture Interrupt Enable
    bitSet(TCCR1B, ICES1);   // Capture on RISING edge
    bitClear(TCCR1B, WGM13); // Enable TCNT1 as CTC mode
    bitSet(TCCR1B, ICNC1);   // Input Capture Noise Canceler
    bitSet(TCCR1B, WGM12);
    bitClear(TCCR1A, WGM11);
    bitClear(TCCR1A, WGM10);
    TCCR1B &= 0xF8; // Set prescaler = 1024
    TCCR1B |= 2;
    OCR1A = TOP - 1;
    bitSet(TIMSK1, OCIE1A); // Timer/Counter1, Output Compare A Match Interrupt Enable
}

void clock_to_time(double clocktime, char timedisp[])
{
    int sechour = 60; // Number of seconds in an hour. Variable for debugging and clock testing purposes.
    // Math functions for converting T1 clock value into hh:mm:ss.ms
    // Note: this does not account for days because I do not expect anyone to run this arduino for a whole ass day
    double hours   = (int)((clocktime / CLOCKF) / sechour) / sechour;
    double minutes = (int)((clocktime / CLOCKF) / sechour - hours * sechour);
    double seconds =      ( clocktime / CLOCKF - (minutes + hours * sechour) * sechour);

    // Create strings for printing time value
    char str_s[6];
    char str_m[3];
    char str_h[3];

    // Change double time values to respective strings
    dtostrf(seconds, 5, 2, str_s);
    dtostrf(minutes, 2, 0, str_m);
    dtostrf(hours, 2, 0, str_h);

    // Merge all strings into one time display string (goofy hardcode)
    int len = 6;
    if(hours > 0)
    {
        len = 0;
        timedisp[0-len] = str_h[0];
        timedisp[1-len] = str_h[1];
        timedisp[2-len] = 'h';
    }
    if(minutes > 0)
    {
        len = 3;
        timedisp[3-len] = str_m[0];
        timedisp[4-len] = str_m[1];
        timedisp[5-len] = 'm';
    }
    timedisp[6-len] = str_s[0];
    timedisp[7-len] = str_s[1];
    timedisp[8-len] = str_s[2];
    timedisp[9-len] = str_s[3];
    timedisp[10-len] = str_s[4];
    timedisp[11-len] = 's';
    timedisp[12-len] = '\0';
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
