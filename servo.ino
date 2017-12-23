#include <avr/interrupt.h>
#define TOP 20000
volatile int cnt = 90;
void setup() {
 DDRB |= 1 << 2; // Pin 10 Output
 DDRB |= 1 << 5 ; // Pin 13 Output
 TCCR1A =(1 << COM1B1);//|(1<<COM1B0); // output OC1B
 TCCR1B = (1<<WGM13) | (1<<CS11); // PFC with prescaler 8
 TCCR1C = 0;
 ICR1 = TOP;
 OCR1B = 10000;
 //TIMSK1 = (1<<2 /*ocieb*/)|
 TIMSK1 = 0;//(1<<0 /*toie*/);
 Serial.begin(57600);

}
void msToServo(double ms){
  int valor = (int)1500*ms;
  OCR1B = valor;
}
void dutycycle(int cycle){
  OCR1B = TOP*(cycle/100);
}
/**
 * Angle in [-90,90]
 */
void angleToServo(double angle){
  msToServo(0.4+((angle + 90)/150));
}
int a = 0;
double ms = 0;
int sumaa = 30;
double sumams = 0.25;
void loop() {
  
  // put your main code here, to run repeatedly:
  angleToServo(a);
  //msToServo(ms);
  //dutycycle(50);
  //analogWrite(10,127);
  
  Serial.println(a);
  
  _delay_ms(1000);
  if(a <= 0){
    sumaa = 90;
//     sumams = 0.25;
  }
  if(a >= 90){
     sumaa = -90;
     sumams = -0.25;
  }
//  
//  if(ms <=0)
    sumams = 0.1;
   if(ms >=2)
    sumams = -0.1;
   a +=sumaa;
   ms += sumams;
} 
ISR(TIMER1_COMPB_vect){
   PORTB ^= (1 <<2);
}
ISR(TIMER1_OVF_vect){
  PORTB ^= (1 << 5);
}

