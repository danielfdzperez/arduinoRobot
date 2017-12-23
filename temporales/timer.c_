#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "common.h"
#include "lcd.h"

volatile long int N = 0;
void setupTimer(int time){
  TCCR1A = 0;
  TCCR1B=0;//N=256
  TCCR1B |=(1<<CS12);
  TCCR1C=0b00000000;
  TIMSK1 |= (1<<OCIE1A);
  OCR1A=31249;// 1 Hz
  //  PRR0 &= ~(1<<5); //Enable clock
  //  sei();
}
void contador(){
  N++;
}

/* ISR(BADISR_vect){ */
/*   N++; */
/* } */

ISR(TIMER1_COMPA_vect){
  //  contador();
  N++;
}

ISR(TIMER1_CAPT_vect,ISR_ALIASOF(TIMER1_COMPA_vect));
ISR(TIMER1_OVF_vect,ISR_ALIASOF(TIMER1_COMPA_vect));

EMPTY_INTERRUPT(BADISR_vect);


int main(){
  configureLCD();
  clear();
  returnHome();
  powerOn();

  setCursor(0,0);
  sendString("Miguel & Daniel");
  N = 0;
  setupTimer(1);
  sei();

  while(1) {
    clear();

    sendInteger(N); // o bien TCNT1
    /* /\* set pin 5 high to turn led on *\/ */
    write(D13,HIGH);
    _delay_ms(250);
    /* set pin 5 low to turn led off */
    writeLow(D13);
    _delay_ms(250);

  }
  return 0;
}
