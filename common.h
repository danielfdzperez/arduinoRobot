#ifndef COMMON_H_
#define COMMON_H_


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
#define PORTX(pin) (port_value[pin])
#define PINX(pin) (pin_value[pin])
#define LOW 0
#define HIGH 1
#define OUTPUT 0
#define INPUT 1


extern const unsigned char pin_bit[NPORTS];
extern volatile unsigned char * const ddr_value[NPORTS];
extern volatile unsigned char * const pin_value[NPORTS];
extern volatile unsigned char * const port_value[NPORTS];
void pinOutput(unsigned char);
void pinInput(unsigned char);
void pinMode(unsigned char,unsigned char);
void writeHigh(unsigned char);
void writeLow(unsigned char);
void write(unsigned char, unsigned char);
unsigned char read(unsigned char);

#endif
