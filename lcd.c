#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "common.h"
#define LCD_START 2 //20ms

#define miWait(c) _delay_ms(c)

volatile int N = 0;
void set4bits(int code){
    if(((code >> 0)&1) == 0)
	writeLow(D4);
    else
	writeHigh(D4);
    if(((code >> 1)&1) == 0)
	writeLow(D5);
    else
	writeHigh(D5);
    if(((code >> 2)&1) == 0)
	writeLow(D6);
    else
	writeHigh(D6);
    if(((code >> 3)&1) == 0)
	writeLow(D7);
    else
	writeHigh(D7);
}
void send(){
    writeHigh(D9);
    writeLow(D9);
}
void send8bits(unsigned char code){
  //Se envía la parte alta:
  set4bits(0x0f & (code >>4));
  send();
  _delay_us(40);
  //Se envía la parte baja:
  set4bits(code & 0x0F);
  send();
    _delay_us(40);
}
void sendCharacter(char character){
    //TODO coger el valor del registro y guardarlo.
    //Hacer un getValue en common
    //int preD8 = D8;
    writeHigh(D8);
    //D8 = preD8;
    send8bits(character);
}
void sendString(char *string){
    int i = 0;
    while(string[i] != '\0'){
	sendCharacter(string[i]);
	miWait(0.05);
	i++;
    }

}

void setCursor(int f, int c){
  int addr;
  addr = f*0x40+c;
  writeLow(D8);
  send8bits(1<<7 | (0x7F & addr));
  _delay_us(40);
  writeLow(D8);
  send8bits(1<<7 | (0x7F & addr));
  _delay_us(37);
}

long int closest10Pow(long int n){
  if(n<10)
    return 1;
  long int i;
  for(i=10;i<=n;i*=10);
  return i/10;

}

void sendInteger(long int ln){
  long int den;
  char chr;
   for(den=closest10Pow(ln);den  != 0 ;den/=10){
    chr = '0' + ((ln/den)%10);
    sendCharacter(chr);
  }

}

int configureLCD(){
    miWait(LCD_START);

    writeLow(D8); 
    set4bits(0b0011);

    send();

    miWait(5);


    send();

    miWait(0.1);

 
    send();

    //miWait(1);
    set4bits(0b0010);
    miWait(0.04);


    send();

    miWait(0.05);


    send();

    set4bits(0b1010);

    send();
    miWait(0.05);
}
void clear(){
    writeLow(D8);
    set4bits(0);
    send();
    set4bits(1);
    send();
    miWait(2);

}

void setupTimer(int time){
  TCCR1A = 0; //WGM
  TCCR1B=0b00001100;//N=256
  TCCR1C=0b00000000;
  TIMSK1 |= (1<<1);
  OCR1A=31249;// 1 Hz
  //  PRR0 &= ~(1<<5); //Enable clock
  //  sei();
}
void contador(){
  write(D13,HIGH);
  N++;
}
EMPTY_INTERRUPT(BADISR_vect);
ISR(TIMER1_COMPA_vect){
  write(D13,HIGH);
  contador();
 }

ISR(TIMER1_CAPT_vect,ISR_ALIASOF(TIMER1_COMPA_vect));
ISR(TIMER1_OVF_vect,ISR_ALIASOF(TIMER1_COMPA_vect));




int main(){
  //    D8 RS
  //    D9 E
  //    DDRC |= (1<<7);
    pinOutput(D7);
    pinOutput(D4);
    pinOutput(D6);
    pinOutput(D5);
    pinOutput(D9);
    pinOutput(D8);

    configureLCD();


    //return home
    set4bits(0b0000);
    send();
    set4bits(0b0010);
    send();
    miWait(2.1);

    //    Power on
    writeLow(D8);
    set4bits(0b0000);
    send();
    set4bits(0b1111);
    send();
    miWait(0.05);

    clear();
    setCursor(0,7);
    sendInteger(1);
    //    Mueve el cursor a la siguiente linea
  
    miWait(0.04);
    setCursor(1,0);  
    sendString("Miguel & Daniel");
    N=1;
    setupTimer(1);
       sei();
    while(1) {
      clear();
      setCursor(0,0);
      sendInteger(TCNT1); // o bien TCNT1
      setCursor(1,0);
      sendInteger(N); // o bien TCNT1
      /* set pin 5 high to turn led on */
      //     write(D13,HIGH);
      //      _delay_ms(250);
      /* set pin 5 low to turn led off */
      //      writeLow(D13);
      _delay_ms(50);
    
        
    }
    return 0;
}
