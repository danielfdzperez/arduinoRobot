#include <avr/io.h>
#include <util/delay.h>
#define LCD_START 20000 //20ms
#define D0 0
#define D1 0
#define D2 0
#define D3 0
#define D4_OUTPUT (DDRD |= (1<<4)
#define D5 0
#define D6 0
#define D7_OUTPUT DDRE |= (1 << 6) //Pin D7 salida
#define D7_INPUT 0
#define D8 0
#define D9 0
#define D11 0
#define miWait(c) _delay_ms(c/100)


/* void miWait(int cicles){ */
/*   PORTC |=(1<<7); */
/*   int i; */
/*   for(i=0;i<cicles*20000;i++); */
/*   PORTC &=~(1<<7); */

/* } */

int configureLCD(){
     miWait(LCD_START);
 
    PORTB &= ~(1<<4); //RS escribir 0
    PORTD |= (1<<4); //D4 escribir 1
    PORTD &= ~(1<<7); //D6 escribir 0
    PORTC |= (1<<6);//D5 1
    PORTE &= ~(1<<6);//D7 0




    PORTB |= (1<<5);
    PORTB &= ~(1<<5);

    miWait(5000);

    PORTB |= (1<<5);
    PORTB &= ~(1<<5);
    miWait(200);
    PORTB |= (1<<5);
    PORTB &= ~(1<<5);
    miWait(1);
    PORTD &= ~(1<<4); //D4 escribir 0
    miWait(1);
    PORTB |= (1<<5);
    PORTB &= ~(1<<5);
    miWait(50);

    PORTB |= (1<<5);
    PORTB &= ~(1<<5);

    PORTE |= (1<<6);//D7 1
    PORTD &= ~(1<<7); //D6 escribir 0
    PORTB |= (1<<5);
    PORTB &= ~(1<<5);
    miWait(40);
}

int main(){
   DDRC |= (1<<7);
    DDRE |= (1 << 6); //Pin D7 salida
    //D7_OUTPUT
    DDRD |= ((1<<4) | (1<<7));// Pin D4 y D6 salida
    DDRC |= (1<<6); //Pin D5 salida
    DDRB |= ((1<<5) | (1<<4));//D9(E) y D8(RS) salida

    configureLCD();
    

    // return home

    PORTB &= ~(1<<4); //RS escribir 0
    
    PORTE &= ~(1<<6);//D7 0
    PORTD &= (1<<7); //D6 escribir 0
    PORTC &= ~(1<<6);//D5 0
    PORTD &= ~(1<<4); //D4 escribir 0

    PORTB |= (1<<5);
    PORTB &= ~(1<<5);

    PORTE &= ~(1<<6);//D7 0
    PORTD &= ~(1<<7); //D6 escribir 0
    PORTC |= (1<<6);//D5 1
    PORTD &= ~(1<<4); //D4 escribir 0

    PORTB |= (1<<5);
    PORTB &= ~(1<<5);

    miWait(3000);

        // display on

    PORTB &= ~(1<<4); //RS escribir 0
    
    PORTE &= ~(1<<6);//D7 0
    PORTD &= ~(1<<7); //D6 escribir 0
    PORTC &= ~(1<<6);//D5 0
    PORTD &= ~(1<<4); //D4 escribir 0

    PORTB |= (1<<5);
    PORTB &= ~(1<<5);

    PORTE |= (1<<6);//D7 1
    PORTD |= (1<<7); //D6 escribir 1
    PORTC |= (1<<6);//D5 1
    PORTD |= (1<<4); //D4 escribir 1

    PORTB |= (1<<5);
    PORTB &= ~(1<<5);

    miWait(40);

    
    
    PORTB |= (1<<4); //RS escribir 1


    PORTE &= ~(1<<6);//D7 0
    PORTD |= (1<<7); //D6 escribir 1
    PORTC &= ~(1<<6);//D5 0
    PORTD &= ~(1<<4); //D4 escribir 0

    PORTB |= (1<<5);
    PORTB &= ~(1<<5);

    PORTE &= ~(1<<6);//D7 0
    PORTD &= ~(1<<7); //D6 escribir 0
    PORTC &= ~(1<<6);//D5 0
    PORTD |= (1<<4); //D4 escribir 1

    PORTB |= (1<<5);
    PORTB &= ~(1<<5);
    while(1) {
      /* set pin 5 high to turn led on */
      PORTC |= (1<<7);
      _delay_ms(1000);

      /* set pin 5 low to turn led off */
      PORTC &= ~(1<<7);
      _delay_ms(1000);
    }
    return 0;
}
