#include <SoftwareSerial.h>
#include <PID_v1.h>
extern "C"{
  #include "common.h"
  #include "lcd.h"
}

#include<avr/io.h>

enum TestadosRobot {botones, buscarPared, girarDerecha, girarIzq, seguirParedIzq, seguirParedDerecha, paredEnfrenteIzq,paredEnfrenteDerecha, sinParedIzq, sinParedDerecha};
enum TestadosRobot estadoRobot = botones;
volatile unsigned int tiempo = 0;//tiempo que se usa en ping y muestrear al principio 10us Timer0

/*
 * BOTONES
 * 
 * 
 */
enum Tboton {right, left, other};
enum Tboton botonPulsado = other;
//enum Tboton botonPulsado = left;
volatile unsigned int muestreo = 0;//Saber cuando muestrear los botones

int analogico = 0;
volatile unsigned char conversionRealizada = 0;


/*Variables para controlar los estados del lcd*/
DatosEnviar datosInternosEnvio;
  

  
/* LCD ASINC
 *    enviado => Se ha enviado la parte baja del mensaje y por ende todo el mensaje
 *    enviando => Se ha enviado la parte alta del mensaje
 *    enviar => Se debe enviar el mensaje
 *    fin => El mensaje ya ha sido enviado
 *    preparado => El LCD ya esta listo para enviar otro mensaje
 *
 */
enum TlcdStatus {enviado, enviando, enviar, fin, preparado};
enum TlcdStatus lcdStatus = enviar;
/*
 *    mover => Se debe mover el cursor
 *    moviendo => Se ha enviado la parte alta de la orden para mover el cursor
 *    movido => Se ha enviado la parte baja de la orden y el cursor se ha movido
 *    quieto => No se esta realizando ninguna accion sobre el cursor
 *
 */
enum TmovimientoCursor {mover, moviendo, movido, quieto};
enum TmovimientoCursor movimientoCursor = quieto;

int cifras = 3;//Cantida de cifras a mostrar
int distancia = 400;//Distancia actual
int distanciaAnterior = 0;//Distancia mostrada anteriormente
int distanciaImprimir = 0;//Distancia que se esta o se debe imprimir
/*-----------------------------------------------------------------*/

/*Variables de control de PING*/
volatile unsigned long tPulsoEcho = 0;//Tiempo para hacer el echo
volatile unsigned int totalPing = 0;//Tiempo total del ping. A los 60ms se debe hacer algo
volatile unsigned int tPulsoPing = 0;//Tiempo del pulso de respuesta de ping

unsigned char echo = 0;//0 no se ha tomado echo. 1 Se ha toamdo echo.

/*
 * PING
 * nada => No se espera nada.
 * recibir => Se espera recibir echo
 * recibido => Se ha recibido toda la señal del echo
 * respuesta => Gestionar la respuesta del echo recibida
 * bloqueo => Bloquea enviar mas peticiones de echo
 * finalizado => Ha esperado lo suficiente para hacer otro ping
 */
enum TpingStatus {nada,recibir,recibido,respuesta,bloqueo, finalizado};
volatile enum TpingStatus recibiendo = nada;
//volatile int recibiendo = 0;//0-> No se espera nada. 1-> Se espera recibir echo. 2-> Echo recibido. 3-> Gestionar respuesta. 4 -> Estado de bloqueo.
volatile int tiempoEcho = 0;//Tiempo de la señal de echo recibida
int pulso = 0;//Si se envio un pulso para peticion de echo
unsigned char triggerPort = D9;//Pin donde esta el trigger
/*------------------------------------------------------------------*/

/*
 * Servo
 * 
 */
 volatile unsigned int tiempoServo = 0;
 unsigned int contadorServo = 0;
 #define TOPSERVO 20000
 enum TestadoServo {derecha = -90, izquierda = 90, centro = 0};
 enum TestadoServo estadoServo = centro;
 enum TestadoServo proximoEstadoServo = centro;
 
 /*------------------------------------------------------------------*/



/* MOTORES Y PID */
// Constantes del PID (Halladas experimentalmente)
#define Ki 0.11
#define Kp 0.099
#define Kd 0.0003

// Pines para control del canal 1,2 del puente h
#define H1_1 1  // A4, PINF
#define H2_1 0  // A5, PINF
#define EN_1 6  //D10, PINB,OC1B
// Pin del ecoder

#define EC_1 1 // D2, PIND, INT1

// Pines para control del canal 3,4 del puente h
#define H1_2 2  // D0, PIND
#define H2_2 6  // D12, PIND
#define EN_2 7  //D11, PINB,OC1C
// Pin del ecoder 

#define EC_2 3 // D3, PIND, INT3

double pulsos_1 = 0;
double antiguo_1 = 0;
double incremento_1 = 0;
double pulsos_1b = 0;
double antiguo_1b = 0;
double incremento_1b = 0;

double ciclo_trabajo_1 = 0;
double ciclo_trabajo_1b = 0;
double ciclo_trabajo_2 = 0;
#define VEL_OBJETIVO_MAX 160
#define VEL_OBJETIVO_SEGUIR 160
#define VEL_OBJETIVO_MIN 100
double velocidad_objetivo = VEL_OBJETIVO_MAX;
double distancia_objetivo = 30;
double distancia_pid = distancia;
double objetivo_1 = velocidad_objetivo;
double objetivo_2 = velocidad_objetivo;

volatile uint16_t * ruedaIzquierda = &OCR1B;
volatile uint16_t * ruedaDerecha = &OCR1C;

#define MAX_TIEMPO_PID 10000 //100ms. 100ms * 1000 = 100000 us. Como el timer son 10us 100000/10 = 10000.
volatile unsigned int tiempo_pid = 0;

PID PID1(&incremento_1,&ciclo_trabajo_1,&velocidad_objetivo, Kp, Ki,Kd,DIRECT);//
PID PID1b(&incremento_1b,&ciclo_trabajo_1b,&velocidad_objetivo, Kp, Ki,Kd,DIRECT);//
double obj = 30;
double actual = 0;
double actuador = 0;
//PID PID2(&actual,&actuador,&obj, 0.01,0.1,0.1,REVERSE);
PID PID2(&distancia_pid,&ciclo_trabajo_2,&distancia_objetivo, 5,0.1,0.001,REVERSE);//
/*------------------------------------------------------------------*/

//Funciones para los motores
int normalizar(double n){
  return (int)( n * (400.0/255.0)) + 275;//
}
int ajuste(double n){
  //return (int)( n * (400.0/255.0))-200;
  return (int)( n * (400.0/100.0))-100;
}

unsigned long now = millis();
unsigned long  lastTime = now;
unsigned long timeChange = (now - lastTime);


void setupMotores(){
  DDRF |= (1 << H1_1) | (1 << H2_1); // Control del puente H a Output
  DDRD |= (1 << H1_2) | (1 << H2_2); // Control del puente H a Output
  DDRB |= (1 << EN_1); // Pin de enable del puente H a Output
  DDRB |= (1 << EN_2); // Pin de enable del puente H a Output
  DDRD &= ~(1<<EC_1); // Pin de lectura del encoder a Input
  DDRD &= ~(1<<EC_2); // Pin de lectura del encoder a Input

  now = millis();
  timeChange = (now - lastTime);
  //Serial.println(timeChange);
  lastTime = now;
  
  PID1.SetMode(AUTOMATIC);
  PID1b.SetMode(AUTOMATIC);
  PID2.SetMode(AUTOMATIC);
  //PID2.SetSampleTime(1);

  //  Serial.begin(57600);
  // Colocar direccion
  PORTF |= (1<<H1_1);
  PORTF &= ~(1 << H2_1);


  PORTD |= (1<<H1_2);
  PORTD &= ~(1 << H2_2);

  /******************************************************/
  /****** Configuración del pin de enable con PWM *******/
  /************** Se utiliza timer 1 ********************/
  /******************************************************/
  /******************************************************/


  /** Modo correcto en fase y frecuencia -> WGM3:0 = 8  */

  //COM1x a 0 cuando subiendo y a 1 cuando bajando->2
  TCCR1A = (1<<COM1C1)|(1<<COM1B1);

  // Modo correcto en fase y frecuencia con TOP en ICRn, prescaler de 1
  TCCR1B = (1<<WGM13)|(1<<CS10);
  TCCR1C = 0;
  ICR1 = 400;
  //TIMSK1 = (1<<OCIE
  /** Ciclo de trabajo del motor a 0% inicialmente*/

  OCR1B = 0;//Rueda Izquierda
  OCR1C = 0;//Rueda derecha

  /** Interrupcion del encoder */
  //EICRA |= (1 << ISC10)|(1<<ISC30); // INT activo con los flancos
  //EIMSK |= (1 << INT1)|(1<<INT3); // Desenmascarar INT
}


//Prepara el LCD
void prepararLCD(){
  configureLCD(&datosInternosEnvio);
  
  clear();
  returnHome();
  powerOn();
  
}

void mensajeBotones(){
  clear();
  setCursor(0,2);
  sendString("Pulsar boton");
  setCursor(1,2);
  sendString("Left");
  setCursor(1,10);
  sendString("Right");
}

void mensajeDistancia(){
  clear();
  setCursor(0,2);
  sendString("Distancia cm");
  setCursor(1,8);
  escribirIzq();
}


void configurarTimer0(){

  //Una interrupcion cada 10us
  //Configurar el reloj
  OCR0A = 79;//10us
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A = (1<<1);// | (1<<COM0A1) | (1<<COM0A0); //Modo CTC
  TCCR0B = 1;//(1<<CS00);// | (1<<CS02);//Sin preescaler
  TIMSK0 = (1<<OCIE0A); 
}

void configurarPing(){
  pinOutput(triggerPort);
  //Habilita interrupciones INT0, para recibir el echo
  EICRA |= 1;
  EIMSK |= 1;
}

void configurarConversorAnalogico(){
 pinInput(A0);
  ADMUX = 0;
  //ADMUX = (1<<REFS1) | (1<<REFS0) | 7; //Uso de 2.35V internos y tomar muestras de A0
  ADMUX = (1<<REFS0) | 7;
  ADCSRA = 0;
  ADCSRB = 0;//(1<<ADTS1) | (1<<ADTS0);
  //ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADSC);//Habilita conversiones, auto trigger, habilita interrupciones
  //ADCSRA = (1<<ADEN);// | (1<<ADIE);
  ADCSRA = (1<<ADEN) | (1<<ADIE) | 7;
  ADCSRA |= (1<<ADSC);
}

void configurarServo(){
 pinOutput(D5);//DDRB |= 1 << 6; // Pin 10 Output
 TCCR3A =(1 << COM3A1); // output OC1B
 TCCR3B = (1<<WGM33) | (1<<CS31); // PFC with prescaler 8
 TCCR3C = 0;
 ICR3 = TOPSERVO;
 //OCR1B = 10000; // Duty Cycle 50%
 OCR3A = 10000;
 TIMSK3 = 0; // Deshabilitar interrupciones del timer
 angleToServo(estadoServo);
 _delay_ms(1000);
 pinInput(D5);
}
/** Procedimiento  msToServo: establece el tiempo del pulso
 * que se envía al servo a ms milisegundos.
 */
void msToServo(double ms){ 
  int valor = (int)1500*ms;
  //OCR1B = valor;
  OCR3A = valor;
}
/** Procedimiento angleToservo: establece el ángulo al que
 * debe moverse y mantener el servo.
 *
 * PRE: Angle en [-90,90]
 */
void angleToServo(double angle){
  msToServo(0.4+((angle + 90)/150));
}

void girarServo(){
  pinOutput(D5);
  switch(estadoRobot){
    
    
    case buscarPared:
    case paredEnfrenteIzq:
    case paredEnfrenteDerecha:
      if(estadoServo != centro){
        //estadoServo = centro;
        //angleToServo(estadoServo);
        proximoEstadoServo = centro;
        angleToServo(proximoEstadoServo);
      }
    break;
    
    case seguirParedIzq:
      if(estadoServo == centro)
        proximoEstadoServo = derecha;
      else if(estadoServo == derecha)
        proximoEstadoServo = centro;
       angleToServo(proximoEstadoServo);
    break;
    
    case seguirParedDerecha:
      if(estadoServo == centro)
        proximoEstadoServo = izquierda;
      else if(estadoServo == izquierda)
        proximoEstadoServo = centro;
      angleToServo(proximoEstadoServo);
    break;

    case girarDerecha:
    case sinParedDerecha:
      if(estadoServo != izquierda){
        proximoEstadoServo = izquierda;
        angleToServo(proximoEstadoServo);
      }
    break;
    
    case girarIzq:
    case sinParedIzq:
      if(estadoServo != derecha){
        proximoEstadoServo = derecha;
        angleToServo(proximoEstadoServo);
      }
    break;
  }
  /*if(estadoRobot == buscarPared){ 
    if(estadoServo != centro){
      estadoServo = centro;
      angleToServo(estadoServo);
    }
    return;
  }
  if(estado)
  if(estadoServo == centro)
    estadoServo = derecha;
  else if(estadoServo == derecha)
    estadoServo = centro;
  angleToServo(estadoServo);*/
}

void servoEnPosicion(){
  pinInput(D5);
  estadoServo = proximoEstadoServo;
}

void setup() {
  Serial.begin(57600);
  //while (!Serial);
  EICRA = 0;
  EIMSK = 0;
  prepararLCD();
  mensajeBotones();
  configurarServo();
  configurarPing();
  //configurarTimer0();
  configurarConversorAnalogico();
  setupMotores();
}

volatile unsigned char e = 0;
ISR(TIMER0_COMPA_vect){
  /*if(conversionRealizada == 0){
    e = 1;
    //Importante esto sirve para que otras interrupciones funcionen. 
    //  Pero si hay muchas cada poco tiempo las distancias salen mal medidas.
    TIMSK0 ^= (1<<OCIE0A);//Deshabilitar interrupciones del timer0
    sei();//Habilitar interrupciones, asi se pueden anidar.
  }*/
  //Incrementa las variables que estan asociadas a este contador
  tiempo_pid ++;
  tiempo ++;
  datosInternosEnvio.tActual ++;
  totalPing ++;
  /*tPulsoEcho ++;
  totalPing ++;
  tPulsoPing ++;
  tiempoServo ++;
  muestreo ++;*/
  /*if(e == 1){
    e = 0;
    //Habilitar otra vez las interrupciones del TIMER0, antes deshabilitar las interrupciones globales.
    cli();
    TIMSK0 ^= (1<<OCIE0A);
  }*/
}

//Captura interrupcion del echo
ISR(INT0_vect){
  if(recibiendo == recibir){
    //Recibe el echo
    //tPulsoEcho = 0;
    tiempo = 0;
    recibiendo = recibido;
  }else
    if(recibiendo == recibido){
      //lo deja de recibir
      //tiempoEcho = tPulsoEcho;
      tiempoEcho = tiempo; 
      recibiendo = respuesta;     
    }
      
}

ISR(ADC_vect){
  //Serial.println("INT");
    conversionRealizada = 1;
}

/**
 * Handler para la interrupción INT1:
 * Incrementar el numero de pulsos de encoder que se han detectado
 */
ISR(INT1_vect){
  //Serial.println("INT");
  pulsos_1++;
}
/**
 * Handler para la interrupción INT2:
 * Incrementar el numero de pulsos de encoder que se han detectado
 */
ISR(INT3_vect){
  //Serial.println("INT");
  pulsos_1b++;
}

void lcdControl(){

  //Si tiene que enviar un mensaje
  if(lcdStatus == enviar){
     distanciaImprimir = sendNumberAsincrono(distanciaImprimir,&datosInternosEnvio);
     lcdStatus = enviando;
     //Serial.println("enviar");
  }
  
  //Parte baja del mensaje a enviar
  if(lcdStatus == enviando && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    distanciaImprimir = sendNumberAsincrono(distanciaImprimir,&datosInternosEnvio);
    lcdStatus = enviado;
    //Serial.println("enviando");
  }

  //Envio todo el mensaje
  if(lcdStatus == enviado && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    lcdStatus = fin;
    cifras --;
    //Serial.println("enviado");
  }

  //Comienza a mover el cursor
  if(movimientoCursor == mover){
    //Serial.println("mover");
    movimientoCursor = moviendo;
    setCursorAsincrono(1, 8, &datosInternosEnvio);
  }

  //Parte baja para mover el cursor
  if(movimientoCursor == moviendo && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    setCursorAsincrono(1, 8, &datosInternosEnvio);
    movimientoCursor = movido;
    //Serial.println("moviendo");
  }

  //Ha movido el cursor
  if(movimientoCursor == movido && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    movimientoCursor = quieto;
    //Serial.println("movido");
  }
  
  //Si ha acabado de enviar un numero y aun quedan cifras envia otro numero
  //Sino mueve el cursor
  if(lcdStatus == fin){
    if(cifras)
      lcdStatus = enviar;
    else{
      movimientoCursor = mover;
      lcdStatus = preparado;
      //Serial.println(lcdStatus);
    } 
  }

  //Si ya envio todo el numero y el cursor esta en su sitio y ha cambiado la distancia envia la nueva distancia
  if(lcdStatus == preparado && movimientoCursor == quieto && distanciaAnterior != distancia){
    cifras = 3;
    distanciaImprimir = distancia;
    distanciaAnterior = distancia;
    lcdStatus = enviar;
  }
}

//Gestion del ping, recibe la direccion de la distancia para que la use el lcd
void pingControl(int * distancia){
  
   //Si tiene que hacer una peticion y no envio el pulso aun
   if(!pulso && recibiendo==nada){
     echo = 0;
    //Comenzar a enviar la peticion de echo
    //PORTB |= (1<<5);
    writeHigh(triggerPort);
    pulso = 1;
    tiempo = 0;
    //tPulsoPing = 0;
    totalPing = 0;
  }

  //Ya comenzo la peticion y debe bajar la señal
  if(pulso && tiempo >= 1/*tPulsoPing >= 1*/){
   
    //Fin peticion echo
    writeLow(triggerPort);//PORTB &= ~(1<<5);
    pulso = 0;
    recibiendo = recibir;
    totalPing = 0;
  }

  //Se ha recibido la respuesta y se debe gestionar
  if(recibiendo == respuesta){
     echo = 1;
    //Echo recibido
    recibiendo = bloqueo;  
    totalPing = 0;
    //Serial.println(tiempo/10);
    *distancia = tiempoEcho/10-2;
    if(*distancia <= 0){
      echo = 0;
      *distancia = 100;
    }
    //Serial.println(*distancia);
    tiempoEcho = 0;
    //tPulsoEcho = 0;
    tiempo = 0;
  }

  //El tiempo del ping fue mayor de 60ms6000
  if(totalPing >= 6000 && recibiendo != finalizado){
    //Fin tiempo de espera para el siguiente echo
    //recibiendo = nada;
    recibiendo = finalizado;
    totalPing = 0;
    //tiempoServo = 0;
    tiempo = 0;
    girarServo();
  }
  
  if(recibiendo == finalizado && tiempo >= 21000/*tiempoServo >= 100000*/){
    if(contadorServo >= 1){
      servoEnPosicion();
      echo = 0;
      //tiempoServo = 0;
      tiempo = 0;
      recibiendo = nada;
      contadorServo = 0;
    }else{
      contadorServo ++;
      tiempo = 0;
    }
  }
}

Tboton valorBoton(unsigned int valor){
  //if((valor == 511 || valor == 255) || (valor == 1023) || (valor <= 1010 && valor >= 1015) || (valor <= 520 && valor >= 500))
  if(valor == 0)
    return right;
  if(valor <= 484 && valor >= 480) 
    return left;
  //return left;
  return other;
}


//PID.SetControllerDirection(DIRECT/REVERSE);
double controlador(){
  double dis = 2*(distancia_pid - distancia_objetivo);
  //Serial.println(distancia_pid);
  //Serial.println(dis);
  //Serial.println();
  if(dis < -100)
    dis = -100;
  if(dis > 100)
    dis = 100;
  return dis;
}
void logicaMotores(){

/*Falta estados para la distancia*/

  if(echo){

    switch(estadoRobot){
      case buscarPared:
      case paredEnfrenteIzq:
      case paredEnfrenteDerecha:
        if(estadoServo == centro)
          distancia_pid = distancia;
        break;
      case girarIzq:
      case seguirParedIzq:
      case sinParedIzq:
        if(estadoServo == derecha){
          distancia_pid = distancia;
          //Serial.println("SDAFSDF");
        }
        break;
      case girarDerecha:
      case seguirParedDerecha:
      case sinParedDerecha:
        if(estadoServo == izquierda)
          distancia_pid = distancia;
        break; 
    }
  }

  if(tiempo_pid >= MAX_TIEMPO_PID){
          tiempo_pid = 0;
  switch(estadoRobot){
    case buscarPared:
        PID1.Compute();
      PID1b.Compute();
      break;
    case seguirParedDerecha:
    case seguirParedIzq:
      PID1.Compute();
      PID1b.Compute();
      PID2.Compute();
      break; 
    case girarIzq:
      PID1b.Compute();
       break;
    case paredEnfrenteIzq:
      PID1b.Compute();
      break;
    case sinParedIzq:
      PID1.Compute();
      break;
    case girarDerecha:
      PID1.Compute();
      break;
    case paredEnfrenteDerecha:
      PID1.Compute();
      break;
    case sinParedDerecha:
      PID1b.Compute();
      break;
      
  }
  }

  actual = distancia_pid * 2;
  //PID1.Compute();
  //PID1b.Compute();
  //Ciclo trabajo_1 PID_1
  //Serial.println("------------------------");

  switch(estadoRobot){
  case buscarPared:
      *ruedaIzquierda = normalizar(ciclo_trabajo_1);   //normalizar(ciclo_trabajo_1);
      *ruedaDerecha = normalizar(ciclo_trabajo_1b);
      break;
  case girarIzq:
    *ruedaIzquierda = 0;//normalizar(ciclo_trabajo_1) + ciclo_trabajo_2;//+ ajuste(ciclo_trabajo_2);
    *ruedaDerecha = normalizar(ciclo_trabajo_1b);
    break;
      case seguirParedIzq:
        //ciclo_trabajo_2 = controlador() + 50;
        *ruedaIzquierda = normalizar(ciclo_trabajo_1); + ajuste(ciclo_trabajo_2);
        *ruedaDerecha = normalizar(ciclo_trabajo_1b);
        break;
  case paredEnfrenteIzq:
        *ruedaIzquierda = 0;//normalizar(ciclo_trabajo_1) + ciclo_trabajo_2;//+ ajuste(ciclo_trabajo_2);
        *ruedaDerecha = normalizar(ciclo_trabajo_1b);
    break;
    
    case sinParedIzq:
      //ciclo_trabajo_2 = 100;
      *ruedaIzquierda = normalizar(ciclo_trabajo_1) + ajuste(ciclo_trabajo_2);
      //Serial.println(ciclo_trabajo_2);
      *ruedaDerecha = 0;//normalizar(ciclo_trabajo_1b);
      break;
        case girarDerecha:
      ciclo_trabajo_2 = 0;
      *ruedaDerecha = 0;//normalizar(ciclo_trabajo_1) + ciclo_trabajo_2;//+ ajuste(ciclo_trabajo_2);
      *ruedaIzquierda = normalizar(ciclo_trabajo_1);
    break;
      case seguirParedDerecha: 
        //ciclo_trabajo_2 = controlador() + 50;
        *ruedaDerecha = normalizar(ciclo_trabajo_1b) + ajuste(ciclo_trabajo_2);
        *ruedaIzquierda = normalizar(ciclo_trabajo_1);
        break;
  case paredEnfrenteDerecha:
        ciclo_trabajo_2 = 0;
        *ruedaDerecha = 0;//normalizar(ciclo_trabajo_1) + ciclo_trabajo_2;//+ ajuste(ciclo_trabajo_2);
        *ruedaIzquierda = normalizar(ciclo_trabajo_1);
    break;
    case sinParedDerecha:
      //ciclo_trabajo_2 = 100;
      *ruedaDerecha = normalizar(ciclo_trabajo_1b) + ajuste(ciclo_trabajo_2);
      //Serial.println(ciclo_trabajo_2);
      *ruedaIzquierda = 0;//normalizar(ciclo_trabajo_1);
      break;
  }

  

  //Cambiar ciclo_trabajo_i mediante el cálculo del PID
  incremento_1 = (pulsos_1 - antiguo_1);
  
  /*Serial.print(pulsos_1);
  Serial.print(" - ");
  Serial.print(antiguo_1);
  Serial.print(" = ");
  Serial.println(incremento_1);*/
  incremento_1b = (pulsos_1b - antiguo_1b);
  
}

void logicaRobot(){
 
   if(estadoRobot == botones && botonPulsado != other){
    //Serial.println("buscar pared");
    mensajeDistancia();

    //Habilitar interrupciones encoders
    EICRA |= (1 << ISC10)|(1<<ISC30); // INT activo con los flancos
    EIMSK |= (1 << INT1)|(1<<INT3); // Desenmascarar INT
    configurarTimer0();
    estadoRobot = buscarPared;
    return;
  }

   if(!echo)
    return;
    
  if(estadoRobot == buscarPared && distancia <= distancia_objetivo+10){
    velocidad_objetivo = VEL_OBJETIVO_MIN;
    //Serial.println("girar");
    if(botonPulsado == right){
      estadoRobot = girarDerecha;
    }
    if(botonPulsado == left){
     // PID2.SetControllerDirection(DIRECT);
      estadoRobot = girarIzq;
    }
    return;
  }

  if(estadoRobot == girarDerecha && distancia <= distancia_objetivo+10 && estadoServo == izquierda){
    velocidad_objetivo = VEL_OBJETIVO_MAX;
    estadoRobot = seguirParedDerecha;
    return;
  }
  if(estadoRobot == girarIzq && distancia <= distancia_objetivo+10 && estadoServo == derecha){
    velocidad_objetivo = VEL_OBJETIVO_MAX;
    //Serial.println("seguir pared izq");
     //PID2.SetControllerDirection(REVERSE);
    estadoRobot = seguirParedIzq;
    return;
  }

  if( estadoRobot == seguirParedIzq && estadoServo == centro && distancia <= distancia_objetivo+10){
    velocidad_objetivo = VEL_OBJETIVO_MIN;
    estadoRobot = paredEnfrenteIzq;
    return;
  }
  if( estadoRobot == seguirParedDerecha && estadoServo == centro && distancia <= distancia_objetivo+10){
    velocidad_objetivo = VEL_OBJETIVO_MIN;
    estadoRobot = paredEnfrenteDerecha;
    return;
  }

  if(estadoRobot == seguirParedIzq && estadoServo == derecha && distancia > distancia_objetivo + 20){
    velocidad_objetivo = VEL_OBJETIVO_MIN;
    estadoRobot = sinParedIzq;
    return;
  }
  
  if(estadoRobot == seguirParedDerecha && estadoServo == izquierda && distancia > distancia_objetivo + 20){
    velocidad_objetivo = VEL_OBJETIVO_MIN;
    estadoRobot = sinParedDerecha;
    return;
  }

  if(estadoRobot == sinParedIzq && distancia <= distancia_objetivo + 20 && estadoServo == derecha){
    velocidad_objetivo = VEL_OBJETIVO_MAX;
    estadoRobot = seguirParedIzq;
    return;
  }
  
  if(estadoRobot == sinParedDerecha && distancia <= distancia_objetivo + 20 && estadoServo == izquierda){
    velocidad_objetivo = VEL_OBJETIVO_MAX;
    estadoRobot = seguirParedDerecha;
    return;
  }

  if(estadoRobot == paredEnfrenteIzq && distancia >= distancia_objetivo+10 && estadoServo == centro){
    velocidad_objetivo = VEL_OBJETIVO_MAX;
    //PID2.SetControllerDirection(REVERSE);
    estadoRobot = seguirParedIzq;
    return;
  }
  if(estadoRobot == paredEnfrenteDerecha && distancia >= distancia_objetivo+10 && estadoServo == centro){
    velocidad_objetivo = VEL_OBJETIVO_MAX;
    estadoRobot = seguirParedDerecha;
    return;
  }
}

void logicaBotones(){
    tiempo = 0;
    analogico = ADC;
    botonPulsado = valorBoton(analogico);
    if(botonPulsado == other){
      conversionRealizada = 0;
      ADCSRA |= (1<<ADSC);
    }
    //Serial.println("ENTRO");
    //Serial.println(conversionRealizada);
    //Serial.println(analogico);
}

void loop() {
  
  if(estadoRobot != botones){
    antiguo_1 = pulsos_1;
    antiguo_1b = pulsos_1b;
  }
  
  if(conversionRealizada && estadoRobot == botones)
    logicaBotones();

    /*Serial.println(distancia_objetivo);
    Serial.println(distancia_pid);
    //Serial.println(actual);
    //Serial.println(obj);
    Serial.println(ciclo_trabajo_2);
    Serial.println(actuador);
    Serial.println("-------------------------------");*/
    /*
    Serial.print(" ");
    Serial.print(*ruedaIzquierda);
    Serial.println();
    //Serial.println(incremento_1);
    Serial.print(incremento_1b);
    Serial.print(" ");
    Serial.print(incremento_1);
    Serial.println();
    //Serial.println(antiguo_1b);
    //Serial.println(estadoRobot);*/
    //Serial.print(ciclo_trabajo_1);
    //Serial.print(" ");
    //Serial.print(ciclo_trabajo_1b);
    //Serial.print(" ");
   /* Serial.println(ciclo_trabajo_2);
    Serial.print(*ruedaIzquierda);
    Serial.print(" ");
    Serial.print(incremento_1);
    Serial.print(" ");
    Serial.print(*ruedaDerecha);
    Serial.print(" ");
    Serial.print(incremento_1b);
    Serial.println();*/
    /*unsigned long now = millis();
    unsigned long timeChange = (now - principio);
     Serial.print(timeChange);*/
   
   // Serial.println();
   // Serial.println("-------------------------------");
    logicaRobot();
  
  
  if(estadoRobot != botones){
    //antiguo_1 = pulsos_1;
    //antiguo_1b = pulsos_1b;
  
    pingControl(&distancia);
    lcdControl();
    logicaMotores();
  }
}
