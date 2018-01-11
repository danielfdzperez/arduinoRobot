// Biblioteca para el control Proporcional, Integral, Diferencial
#include <PID_v1.h>

// Constantes del PID (Halladas experimentalmente)
#define Ki 0.11
#define Kp 0.099
#define Kd 0.0003

// Para que la señal pwm que le llega al motor sea de 20Hz,
// el maximo debe ser de 200.
#define TOP_MOTOR 200

// Pines para control del primer canal del puente h
#define H1_1 1  // A4, PINF
#define H2_1 0  // A5, PINF
#define EN_1 7  //D13, PINC
// Pin del ecoder

#define EC_1 1 // D2, PIND, INT1

// Pines para control del segundo canal del puente h
#define H1_2 2  // D0, PIND
#define H2_2 6  // D12, PIND
#define EN_2 7  //D6, PIND

// Pin del ecoder

#define EC_2 3 // D2, PIND, INT1


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

  /******************************************************/
  /************Inicialización de los PID ****************/
  /******************************************************/
  /*
    queremos controlar el incremento_i cambiando
     ciclo_trabajo_i
  */
PID PID1(&incremento_1,&ciclo_trabajo_1,&objetivo_1, Kp, Ki,Kd,DIRECT);
PID PID2(&incremento_2,&ciclo_trabajo_2,&objetivo_2, Kp, Ki,Kd,DIRECT);


int normalizar(double n){
  return (int)( n * (200.0/255.0)) + 100;//
}

void setup(){
  DDRF |= (1 << H1_1) | (1 << H2_1); // Control del puente H a Output
  DDRB |= (1 << EN_1); // Pin de enable del puente H a Output


  // Pines de lectura de los encoder a Input,
  // control del puente H y enable a Output
  DDRD = ~(1<<EC_1) & ~(1<<EC_2) | (1 << EN_2) | (1 << H1_2) | (1 << H2_2);
  PID1.SetMode(AUTOMATIC);
  PID2.SetMode(AUTOMATIC);
  Serial.begin(57600);
  // Colocar direccion
  PORTF |= (1<<H1_1);
  PORTF &= ~(1 << H2_1);

  PORTD |= (1<<H1_2);
  PORTD &= ~(1 << H2_2);

  /******************************************************/
  /****** Configuración del pin de enable con PWM *******/
  /************** Se utiliza timer 4 ********************/
  /******************************************************/
  /******************************************************/


  /** Modo correcto en fase y frecuencia -> WGM4 = 1  */

  TCCR4A = (1 <<COM4A1);  // output OC4A (D13)
  TCCR4B = (1<<CS41);     // prescaler 8
  TCCR4C =(1 << COM4D1);  // output OC4D (D6)
  TCCR4D = (1<<WGM40);    // Modo correcto en fase y frecuencia
  TCCR4E = 0;

  //OCR4C define el maximo al que llega el timer
  OCR4C = TOP_MOTOR;

  /** Ciclo de trabajo del motor a 0% inicialmente*/

  OCR4A = 0;
  OCR4D = 0;
  //  OCR1C = 0;

  /** Interrupcion de los encoder */

  EICRA = (1 << ISC10) | (1 << ISC30); // INT1 e INT3 activos con los flancos
  EIMSK = (1 << INT1) | (1 << INT3); // Desenmascarar INT1 e INT3





}

void loop(){
  antiguo_1 = pulsos_1;
  antiguo_2 = pulsos_2;

  //  Serial.println(incremento_1);

  /*
   * Actualizar los valores de ciclo_trabajo_i mediante el calculo del PID
   */
  PID1.Compute();
  PID2.Compute();

  // Actualizar los timer
  OCR4A = normalizar(ciclo_trabajo_1);
  OCR4D = normalizar(ciclo_trabajo_2);

  // Esperar
  _delay_ms(300);

  // Actualizar las variables de control de los PID
  incremento_1 = (pulsos_1 - antiguo_1);
  incremento_2 = (pulsos_2 - antiguo_2);
}


/**
 * Handler para la interrupción INT1
 * Incrementa el número de pulsos de encoder registrados
 */
ISR(INT1_vect){
  pulsos_1++;
}
/**
 * Handler para la interrupción INT3
 * Incrementa el número de pulsos de encoder registrados
 */
ISR(INT3_vect){
  pulsos_2++;
}
