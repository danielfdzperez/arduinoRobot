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
	writeLow(A3);
    else
	writeHigh(A3);
    if(((code >> 2)&1) == 0)
	writeLow(A2);
    else
	writeHigh(A2);
    if(((code >> 3)&1) == 0)
	writeLow(D7);
    else
	writeHigh(D7);
}
void lcdSend(){
    writeHigh(A1);
    writeLow(A1);
}
void send8bits(unsigned char code){
  //Se envía la parte alta:
  set4bits(0x0f & (code >>4));
  lcdSend();
  _delay_us(40);
  //Se envía la parte baja:
  set4bits(code & 0x0F);
  lcdSend();
    _delay_us(40);
}

void sen8bitsAsincrono(unsigned char code, DatosEnviar * estructura){
  if(estructura->parte == alta){
      set4bits(0x0f & (code >>4));
      lcdSend();
      estructura->parte = baja;
  }else{
      //Se envía la parte baja:
      set4bits(code & 0x0F);
      lcdSend();
      estructura->parte = alta;
  }
  estructura->espera = 40;
  estructura->tActual = 0;
}

int sendNumberAsincrono(unsigned int code, DatosEnviar * estructura){
  int ret = code;
  if(estructura->parte == baja)
    ret = code / 10;
  writeHigh(D8);
  sen8bitsAsincrono('0' + (code % 10),estructura);
    
  return ret;
}

int setCursorAsincrono(int f, int c, DatosEnviar * estructura){
  int ret = 0;
  if(estructura->parte == baja)
    ret = 1;
  int addr;
  addr = f*0x40+c;
  writeLow(D8);
  sen8bitsAsincrono(1<<7 | (0x7F & addr), estructura);
  return ret;
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
  //_delay_us(40);
  //writeLow(D8);
  //send8bits(1<<7 | (0x7F & addr));
  //_delay_us(37);
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

int configureLCD(DatosEnviar * estructura){

  estructura->parte = alta;
  estructura->espera = 0;
  estructura->tActual = 0;

  pinOutput(D7);
  pinOutput(D4);
  pinOutput(A2);
  pinOutput(A3);
  pinOutput(A1);
  pinOutput(D8);

  miWait(LCD_START);

  writeLow(D8); 
  set4bits(0b0011);

  lcdSend();

  miWait(5);


  lcdSend();

  miWait(0.1);

 
  lcdSend();

  //miWait(1);
  set4bits(0b0010);
  miWait(0.04);


  lcdSend();

  miWait(0.05);


  lcdSend();

  set4bits(0b1010);

  lcdSend();
  miWait(0.05);

}
void returnHome(){
  writeLow(D8);
  set4bits(0b0000);
  lcdSend();
  set4bits(0b0010);
  lcdSend();
  miWait(2.1);

}

void escribirIzq(){
  writeLow(D8);
  set4bits(0);
  lcdSend();
  set4bits(0b0100);
  lcdSend();
  miWait(0.05);
}

void powerOn(void){
  writeLow(D8);
  set4bits(0b0000);
  lcdSend();
  set4bits(0b1100);//Sin blink de cursor. Poner 1 al final para blink.
  lcdSend();
  miWait(0.05);

}
void clear(){
  writeLow(D8);
  set4bits(0);
  lcdSend(); 
  set4bits(1);
  lcdSend();
  miWait(2);

}
