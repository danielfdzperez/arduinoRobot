
#include<avr/io.h>
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


void pinOutput(unsigned char pin){
    *DDRX(pin) |= PIN_BIT(pin);
}

void pinInput(unsigned char pin){
    *DDRX(pin) &= (~PIN_BIT(pin));
}
void pinMode(unsigned char pin,unsigned char mode){
    switch(mode){
	case INPUT:
	    pinInput(pin);
	    break;
	case OUTPUT:
	    pinOutput(pin);
	    break;
    }
}


void writeHigh(unsigned char pin){
    *PORTX(pin) |= PIN_BIT(pin);
}
void writeLow(unsigned char pin){
    *PORTX(pin) &= (~PIN_BIT(pin));
}
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

unsigned char read(unsigned char pin){
    return *PINX(pin) & PIN_BIT(pin);
}
