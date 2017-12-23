
#include<avr/io.h>
#include "Arduino.h"  
#include "common.h"

const unsigned char pin_bit[NPORTS]=
{ D0_BIT,D1_BIT,D2_BIT,D3_BIT,D4_BIT,D5_BIT,D6_BIT,D7_BIT,D8_BIT,
    D9_BIT,D10_BIT,D11_BIT,D12_BIT,D13_BIT,A0_BIT,A1_BIT,A2_BIT,A3_BIT,A4_BIT,A5_BIT
};
volatile unsigned char * const ddr_value[NPORTS]={&DDRD,&DDRD,&DDRD,&DDRD,&DDRD,&DDRC,&DDRD, &DDRE, &DDRB, &DDRB, &DDRB, &DDRB, &DDRD,
&DDRC, &DDRF, &DDRF, &DDRF, &DDRF, &DDRF, &DDRF};
volatile unsigned char * const pin_value[NPORTS]={&PIND,&PIND,&PIND,&PIND,&PIND,&PINC,&PIND, &PINE, &PINB, &PINB, &PINB, &PINB, &PIND,
&PINC, &PINF, &PINF, &PINF, &PINF, &PINF, &PINF};
volatile unsigned char * const port_value[NPORTS]={&PORTD,&PORTD,&PORTD,&PORTD,&PORTD,&PORTC,&PORTD, &PORTE, &PORTB, &PORTB, &PORTB, &PORTB, &PORTD,&PORTC, &PORTF, &PORTF, &PORTF, &PORTF, &PORTF, &PORTF};


/*
 * Pone un pin en modo OUTPUT
 */
void pinOutput(unsigned char pin){
    *DDRX(pin) |= PIN_BIT(pin);
}

/*
 * Pine un pin en modo INPUT
 */
void pinInput(unsigned char pin){
    *DDRX(pin) &= (~PIN_BIT(pin));
}

/*
 * Selecciona en que modo poner un pin.
 *
 * pin => pin al que cambiar el modo
 * mode => Modo en que se pone INPUT o OUTPUT
 */
void pinModeN(unsigned char pin,unsigned char mode){
    switch(mode){
	case INPUT:
	    pinInput(pin);
	    break;
	case OUTPUT:
	    pinOutput(pin);
	    break;
    }
}


/*
 *
 * Pone a alto el pin deseado
 */
void writeHigh(unsigned char pin){
    *PORTX(pin) |= PIN_BIT(pin);
}

/*
 *
 * Pone a bajo el pin deseado
 *
 */
void writeLow(unsigned char pin){
    *PORTX(pin) &= (~PIN_BIT(pin));
}

/*
 *
 * Pone a alto o a bajo un pin
 *
 * pin => El pin que debe ponerse a alto
 * mode => Modo HIGH o LOW
 */
void write(unsigned char pin,unsigned char mode){
    switch(mode){
	case LOW:
	    writeLow(pin);
	    break;
	case HIGH:
	    writeHigh(pin);
	    break;
    }
}

//Lee el valor que hay en uno de los pines
unsigned char read(unsigned char pin){
    return *PINX(pin) & PIN_BIT(pin);
}
