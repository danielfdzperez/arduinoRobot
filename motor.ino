#include <PID_v1.h>
// Constantes del PID

#define Ki 0.11
#define Kp 0.099
#define Kd 0.0003

// Pines para control del puente h
#define H1 1  // A4, PINF
#define H2 0  // A5, PINF
#define EN 6//D10, PINB
// Pin del ecoder

#define EC 1 // D2, PIND, INT1

 double pulsos_1 = 0;
 double antiguo_1 = 0;
 double incremento_1 = 0;
 double pulsos_2 = 0;
double antiguo_2 = 0;
double incremento_2 = 0;

double ciclo_trabajo_1 = 0;
 double ciclo_trabajo_2 = 0;
 double objetivo_2 = 0;
 double objetivo_1 = 690;


PID PID1(&incremento_1,&ciclo_trabajo_1,&objetivo_1, Kp, Ki,Kd,DIRECT);
PID PID2(&incremento_2,&ciclo_trabajo_2,&objetivo_2, Kp, Ki,Kd,DIRECT);
int normalizar(double n){
  return (int)( n * (400.0/255.0)) + 275;//
}

void setup(){
  DDRF |= (1 << H1) | (1 << H2); // Control del puente H a Output
  DDRB |= (1 << EN); // Pin de enable del puente H a Output
  DDRD &= ~(1<<EC); // Pin de lectura del encoder a Input
  PID1.SetMode(AUTOMATIC);
  PID2.SetMode(AUTOMATIC);
  Serial.begin(57600);
  // Colocar direccion
  PORTF |= (1<<H1);
  PORTF &= ~(1 << H2);

  /******************************************************/
  /****** Configuración del pin de enable con PWM *******/
  /************** Se utiliza timer 1 ********************/
  /******************************************************/
  /******************************************************/


  /** Modo correcto en fase y frecuencia -> WGM3:0 = 8  */


  TCCR1A = (1<<COM1B1)|(1<<COM1C1); //COM1x a 0 cuando subiendo y a 1 cuando bajando->2
  TCCR1B = (1<<WGM13)|(1<<CS10); // Modo correcto en fase y frecuencia con TOP en ICRn, prescaler de 1
  TCCR1C = 0;
  ICR1 = 400;
  //TIMSK1 = (1<<OCIE
  /** Ciclo de trabajo del motor a 0% inicialmente*/

  OCR1B = 0;
  //  OCR1C = 0;

  /** Interrupcion del encoder */

  EICRA = (1 << ISC10); // INT1 activo con los flancos
  EIMSK = (1 << INT1); // Desenmascarar INT1

  /******************************************************/
  /***************Inicialización de  PID ****************/
  /******************************************************/
   /*
    queremos controlar el incremento_i cambiando
     ciclo_trabajo_i
  */



}

void loop(){
  antiguo_1 = pulsos_1;
  // ...

  Serial.println(incremento_1);
  //ciclo_trabajo_1 +=10;
  PID1.Compute();
  PID2.Compute();
  OCR1B = normalizar(ciclo_trabajo_1);   //normalizar(ciclo_trabajo_1);
  //PORTB |= (1<<EN);
  //digitalWrite(9,HIGH);
  _delay_ms(300);
  incremento_1 = (pulsos_1 - antiguo_1);
}


/**
 * Handler para la interrupción INT1
 */
ISR(INT1_vect){
  pulsos_1++;
}

ISR(INT3_vect){
  pulsos_2++;
}
