#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "common.h"
#include "lcd.h"
#define LCD_START 2 //20ms

#define miWait(c) _delay_ms(c)


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
  pinOutput(D7);
  pinOutput(D4);
  pinOutput(D6);
  pinOutput(D5);
  pinOutput(D9);
  pinOutput(D8);

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
void returnHome(){
    set4bits(0b0000);
  send();
  set4bits(0b0010);
  send();
  miWait(2.1);

}

void powerOn(void){
  writeLow(D8);
  set4bits(0b0000);
  send();
  set4bits(0b1111);
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
