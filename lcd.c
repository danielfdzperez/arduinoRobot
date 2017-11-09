#include <avr/io.h>
#include <util/delay.h>
#define LCD_START 20000 //20ms
/* #define D0 0 */
/* #define D1 0 */
/* #define D2 0 */
/* #define D3 0 */
/* #define D4_OUTPUT (DDRD |= (1<<4) */
/* #define D5 0 */
/* #define D6 0 */
/* #define D7_OUTPUT DDRE |= (1 << 6) //Pin D7 salida */
/* #define D7_INPUT 0 */
/* #define D8 0 */
/* #define D9 0 */
/* #define D11 0 */

#define PIN_D4 PORTD
#define PIN_D5 PORTC
#define PIN_D6 PORTD
#define PIN_D7 PORTE
#define PIN_RS PORTB
#define PIN_E PORTB

#define BIT_D4 4
#define BIT_D5 6
#define BIT_D6 7
#define BIT_D7 7
#define BIT_RS 4
#define BIT_E 5

#define D4_OUTPUT (DDRD |= (1<<4)
#define D7_OUTPUT DDRE |= (1 << 6) //Pin D7 salida

#define BSET(PIN,BIT) PIN |=(1<<BIT)
#define BCLR(PIN,BIT) PIN =(1<<BIT)

#define miWait(c) _delay_ms(c/100)


/* void miWait(int cicles){ */
/*   PORTC |=(1<<7); */
/*   int i; */
/*   for(i=0;i<cicles*20000;i++); */
/*   PORTC &=~(1<<7); */

/* } */

/* int configureLCD(){ */
/*      miWait(LCD_START); */
 
/*     PORTB &= ~(1<<4); //RS escribir 0 */
/*     PORTD |= (1<<4); //D4 escribir 1 */
/*     PORTD &= ~(1<<7); //D6 escribir 0 */
/*     PORTC |= (1<<6);//D5 1 */
/*     PORTE &= ~(1<<6);//D7 0 */




/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */

/*     miWait(5000); */

/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */
/*     miWait(200); */
/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */
/*     miWait(1); */
/*     PORTD &= ~(1<<4); //D4 escribir 0 */
/*     miWait(1); */
/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */
/*     miWait(50); */

/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */

/*     PORTE |= (1<<6);//D7 1 */
/*     PORTD &= ~(1<<7); //D6 escribir 0 */
/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */
/*     miWait(40); */
/* } */

/* int main(){ */
/*    DDRC |= (1<<7); */
/*     DDRE |= (1 << 6); //Pin D7 salida */
/*     //D7_OUTPUT */
/*     DDRD |= ((1<<4) | (1<<7));// Pin D4 y D6 salida */
/*     DDRC |= (1<<6); //Pin D5 salida */
/*     DDRB |= ((1<<5) | (1<<4));//D9(E) y D8(RS) salida */

/*     configureLCD(); */
    

/*     // return home */

/*     PORTB &= ~(1<<4); //RS escribir 0 */
    
/*     PORTE &= ~(1<<6);//D7 0 */
/*     PORTD &= (1<<7); //D6 escribir 0 */
/*     PORTC &= ~(1<<6);//D5 0 */
/*     PORTD &= ~(1<<4); //D4 escribir 0 */

/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */

/*     PORTE &= ~(1<<6);//D7 0 */
/*     PORTD &= ~(1<<7); //D6 escribir 0 */
/*     PORTC |= (1<<6);//D5 1 */
/*     PORTD &= ~(1<<4); //D4 escribir 0 */

/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */

/*     miWait(3000); */

/*         // display on */

/*     PORTB &= ~(1<<4); //RS escribir 0 */
    
/*     PORTE &= ~(1<<6);//D7 0 */
/*     PORTD &= ~(1<<7); //D6 escribir 0 */
/*     PORTC &= ~(1<<6);//D5 0 */
/*     PORTD &= ~(1<<4); //D4 escribir 0 */

/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */

/*     PORTE |= (1<<6);//D7 1 */
/*     PORTD |= (1<<7); //D6 escribir 1 */
/*     PORTC |= (1<<6);//D5 1 */
/*     PORTD |= (1<<4); //D4 escribir 1 */

/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */

/*     miWait(40); */

    
    
/*     PORTB |= (1<<4); //RS escribir 1 */


/*     PORTE &= ~(1<<6);//D7 0 */
/*     PORTD |= (1<<7); //D6 escribir 1 */
/*     PORTC &= ~(1<<6);//D5 0 */
/*     PORTD &= ~(1<<4); //D4 escribir 0 */

/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */

/*     PORTE &= ~(1<<6);//D7 0 */
/*     PORTD &= ~(1<<7); //D6 escribir 0 */
/*     PORTC &= ~(1<<6);//D5 0 */
/*     PORTD |= (1<<4); //D4 escribir 1 */

/*     PORTB |= (1<<5); */
/*     PORTB &= ~(1<<5); */
/*     while(1) { */
/*       /\* set pin 5 high to turn led on *\/ */
/*       PORTC |= (1<<7); */
/*       _delay_ms(1000); */

/*       /\* set pin 5 low to turn led off *\/ */
/*       PORTC &= ~(1<<7); */
/*       _delay_ms(1000); */
/*     } */
/*     return 0; */
/* } */
void lcd_sendEnable(void){
  CLRCTRL(BITE);
  //Es posible que haya que meter un wait
  myWait(10);
  SETCTRL(BITE);
  myWait(10);
  //Es posible que haya que meter un wait
  CLRCTRL(BITE);
  myWait(10);
}
void lcd_send4bitCode( char code){ //La parte más significativa de code irá al pin más significativo
  int i;
  if(((code >> 0)&1) == 0)
    CLRD(BITD4);
  else
    SETD(BITD4);
  if(((code >> 1)&1) == 0)
    CLRD(BITD5);
  else
    SETD(BITD5);
    if(((code >> 2)&1) == 0)
    CLRD(BITD6);
  else
    SETD(BITD6);
  if(((code >> 3)&1) == 0)
    CLRD(BITD7);
  else
    SETD(BITD7);

  lcd_sendEnable();
}
void lcd_send8bitCode(unsigned char code){
  //Primero se envía la parte alta:
  lcd_send4bitCode(code >>4);
  //Después se envía la parte baja:
  lcd_send4bitCode(code & 0x0F);
}

void lcd_setup(void){
  myWait(40000);
  CLRCTRL(BITRS);
  CLRCTRL(BITE);
  CLRD(BITD7);
  CLRD(BITD6);
  SETD(BITD5);
  SETD(BITD4);
  lcd_sendEnable();
  myWait(5000);
  lcd_sendEnable();
  myWait(500);
  lcd_sendEnable();
  CLRD(BITD4);
  lcd_sendEnable();

  lcd_send8bitCode(0x28);//Function set: 4 bit interface,2 lines,small font
  myWait(50);
  lcd_send8bitCode(0x0D); //Display on
  myWait(50);
  lcd_send8bitCode(0x01); // Clear display
  myWait(5000);
  lcd_send8bitCode(0x04); // Entry mode set: increment cursor, no shift
  myWait(50);
}

void lcd_printchr(char chr, char addr){
  CLRCTRL(BITRS);
  lcd_send8bitCode(1<<7 | (0x7F & addr));
  SETCTRL(BITRS);
  lcd_send8bitCode(chr);
}
void lcd_prints(char * ln,char addr){
  int i;
  for(i=0;ln[i] != '\0';i++){
    lcd_printchr(ln[i],addr+i);
  }
}

int main(void){
   DDRB |= _BV(DDB5);
   //  setPinsAsOutput();
   DDRC |= (1<<7);
   DDRE |= (1 << 6); //Pin D7 salida
    //D7_OUTPUT
   DDRD |= ((1<<4) | (1<<7));// Pin D4 y D6 salida
   DDRC |= (1<<6); //Pin D5 salida
   DDRB |= ((1<<5) | (1<<4));//D9(E) y D8(RS) salida

  lcd_setup();
  lcd_prints("Hola\0",0);
   while(1) {
  /* set pin 5 high to turn led on */
  PORTB |= _BV(PORTB5);
  _delay_ms(1000);

  /* set pin 5 low to turn led off */
  PORTB &= ~_BV(PORTB5);
  _delay_ms(1000);
 }
}
