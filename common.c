#include<avr/io.h>

#define D0 0
#define D1 1
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define D8 8
#define D9 9
#define D10 10
#define D11 11
#define D12 12
#define D13 13
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define NPORTS 20

#define D0_BIT (1 << 2)
#define D1_BIT (1 << 3)
#define D2_BIT (1 << 1)
#define D3_BIT (1 << 0)
#define D4_BIT (1 << 4)
#define D5_BIT (1 << 6)
#define D6_BIT (1 << 7)
#define D7_BIT (1 << 6)
#define D8_BIT (1 << 4)
#define D9_BIT (1 << 5)
#define D10_BIT (1 << 6)
#define D11_BIT (1 << 7)
#define D12_BIT (1 << 6)
#define D13_BIT (1 << 7)
#define A0_BIT (1 << 7)
#define A1_BIT (1 << 6)
#define A2_BIT (1 << 5)
#define A3_BIT (1 << 4)
#define A4_BIT (1 << 1)
#define A5_BIT (1 << 0)
#define ERROR 256
#define PIN_BIT(pin) (pin_bit[pin])
#define DDRX(pin) (ddr_value[pin])
#define LOW 0
#define HIGH 1


const unsigned char pin_bit[NPORTS]=
{ D0_BIT,D1_BIT,D2_BIT,D3_BIT,D4_BIT,D5_BIT,D6_BIT,D7_BIT,D8_BIT,
    D9_BIT,D10_BIT,D11_BIT,D12_BIT,D13_BIT,A0_BIT,A1_BIT,A2_BIT,A3_BIT,A4_BIT,A5_BIT
};

volatile unsigned char * const ddr_value[NPORTS]={&DDRD,&DDRD,&DDRD,&DDRD,&DDRD,&DDRC,&DDRD, &DDRE, &DDRB, &DDRB, &DDRB, &DDRB, &DDRD,
&DDRC, &DDRF, &DDRF, &DDRF, &DDRF, &DDRF, &DDRF};

void pinHigh(unsigned char pin){
    *DDRX(pin) |= PIN_BIT(pin);
}

void pinLow(unsigned char pin){
    *DDRX(pin) &= (~PIN_BIT(pin));
}
void pinMode(unsigned char pin,unsigned char mode){
    switch(mode){
	case LOW:
	    pinLow(pin);
	    break;
	case HIGH:
	    pinHigh(pin);
	    break;
    }
}

int main(){
    pinMode(D0,LOW);
}
