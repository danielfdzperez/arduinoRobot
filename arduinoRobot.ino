#include <SoftwareSerial.h>
extern "C"{
  #include "common.h"
  #include "lcd.h"
}

#include<avr/io.h>

enum TestadosRobot {botones, buscarPared, girarDerecha, girarIzq, seguirParedIzq, seguirParedDerecha, paredEnfrente, sinParedIzq, sinParedDerecha};
enum TestadosRobot estadoRobot = botones;
volatile unsigned int tiempo = 0;//tiempo que se usa en ping y muestrear al principio 10us Timer0

/*
 * BOTONES
 * 
 * 
 */
enum Tboton {right, left, other};
enum Tboton botonPulsado = other;
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
 
 /*------------------------------------------------------------------*/

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
  EICRA = 1;
  EIMSK = 1;
  
}

void configurarConversorAnalogico(){
  ADMUX = 0;
  ADMUX = (1<<REFS1) | (1<<REFS0) | 7; //Uso de 2.35V internos y tomar muestras de A0

  ADCSRA = 0;
  ADCSRB = 0;//(1<<ADTS1) | (1<<ADTS0);
  //ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADSC);//Habilita conversiones, auto trigger, habilita interrupciones
  ADCSRA = (1<<ADEN) | (1<<ADIE);
  ADCSRA |= (1<<ADSC);
}

void configurarServo(){
 pinOutput(D10);//DDRB |= 1 << 6; // Pin 10 Output
 TCCR1A =(1 << COM1B1); // output OC1B
 TCCR1B = (1<<WGM13) | (1<<CS11); // PFC with prescaler 8
 TCCR1C = 0;
 ICR1 = TOPSERVO;
 OCR1B = 10000; // Duty Cycle 50%
 TIMSK1 = 0; // Deshabilitar interrupciones del timer
 angleToServo(estadoServo);
 _delay_ms(1000);
 pinInput(D10);
}
/** Procedimiento  msToServo: establece el tiempo del pulso
 * que se envía al servo a ms milisegundos.
 */
void msToServo(double ms){ 
  int valor = (int)1500*ms;
  OCR1B = valor;
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
  //Serial.println(estadoServo);
  switch(estadoRobot){
    case buscarPared:
    case paredEnfrente:
      if(estadoServo != centro){
        estadoServo = centro;
        angleToServo(estadoServo);
      }
    break;
    
    case seguirParedIzq:
      if(estadoServo == centro)
        estadoServo = izquierda;
      else if(estadoServo == izquierda)
        estadoServo = centro;
      angleToServo(estadoServo);
    break;
    
    case seguirParedDerecha:
      if(estadoServo == centro)
        estadoServo = derecha;
      else if(estadoServo == derecha)
        estadoServo = centro;
      angleToServo(estadoServo);
    break;

    case girarDerecha:
    case sinParedDerecha:
      if(estadoServo != derecha){
        estadoServo = derecha;
        angleToServo(estadoServo);
      }
    break;
    case girarIzq:
    case sinParedIzq:
      if(estadoServo != izquierda){
        estadoServo = izquierda;
        angleToServo(estadoServo);
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

void setup() {
  Serial.begin(57600);
  while (!Serial);
  prepararLCD();
  mensajeBotones();
  configurarServo();
  configurarPing();
  configurarTimer0();
  configurarConversorAnalogico();
}

volatile unsigned char e = 0;
ISR(TIMER0_COMPA_vect){
  if(conversionRealizada == 0){
    e = 1;
    //Importante esto sirve para que otras interrupciones funcionen. 
    //  Pero si hay muchas cada poco tiempo las distancias salen mal medidas.
    TIMSK0 ^= (1<<OCIE0A);//Deshabilitar interrupciones del timer0
    sei();//Habilitar interrupciones, asi se pueden anidar.
  }
  //Incrementa las variables que estan asociadas a este contador
  tiempo ++;
  datosInternosEnvio.tActual ++;
  totalPing ++;
  /*tPulsoEcho ++;
  totalPing ++;
  tPulsoPing ++;
  tiempoServo ++;
  muestreo ++;*/
  if(e == 1){
    e = 0;
    //Habilitar otra vez las interrupciones del TIMER0, antes deshabilitar las interrupciones globales.
    cli();
    TIMSK0 ^= (1<<OCIE0A);
  }
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
    conversionRealizada = 1;
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
     
    //Echo recibido
    recibiendo = bloqueo;  
    totalPing = 0;
    //Serial.println(tiempo/10);
    *distancia = tiempoEcho/10-2;
    Serial.println(*distancia);
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
    pinOutput(D10);
    girarServo();
  }
  if(recibiendo == finalizado && tiempo >= 30000/*tiempoServo >= 100000*/){
    if(contadorServo >= 2){
      pinInput(D10);
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
  if(valor >= 490 && valor <= 500)
    return right;
  if(valor <= 1014 && valor >= 990)
    return left;
  return other;
}

void logicaRobot(){
   if(estadoRobot == botones && botonPulsado != other){
    mensajeDistancia();
    estadoRobot = buscarPared;
    return;
  }

  if(estadoRobot == buscarPared && distancia <= 30){
    if(botonPulsado == right)
      estadoRobot = girarDerecha;
    if(botonPulsado == left)
      estadoRobot = girarIzq;
    return;
  }

  if(estadoRobot == girarDerecha && distancia >= 30){
    estadoRobot = seguirParedDerecha;
    return;
  }
  if(estadoRobot == girarIzq && distancia >= 30){
    estadoRobot = seguirParedIzq;
    return;
  }

  if( (estadoRobot == seguirParedIzq || estadoRobot == seguirParedDerecha) && estadoServo == centro && distancia <= 30){
    estadoRobot = paredEnfrente;
    return;
  }

  if(estadoRobot == seguirParedIzq && estadoServo == izquierda && distancia > 40){
    estadoRobot == sinParedIzq;
    return;
  }
  
  if(estadoRobot == seguirParedDerecha && estadoServo == derecha && distancia > 40){
    estadoRobot == sinParedDerecha;
    return;
  }

  if(estadoRobot == sinParedIzq && distancia <= 40){
    estadoRobot = seguirParedIzq;
    return;
  }
  
  if(estadoRobot == sinParedDerecha && distancia <= 40){
    estadoRobot = seguirParedDerecha;
    return;
  }
}

void loop() {

  //Los botones no hay que mirarlos todo el rato porque solo sera al principio.
  //Esta puesto las interrupciones del TIMER0 pero no es necesario todo lo que le he puesto. Porque se hara por estados del robot.
 
  if(conversionRealizada && tiempo > 30000 && estadoRobot == botones){
    tiempo = 0;
    analogico = ADC;
    botonPulsado = valorBoton(analogico);
    if(botonPulsado == other){
      conversionRealizada = 0;
      ADCSRA |= (1<<ADSC);
    }
    //Serial.println("ENTRO");
    //Serial.println(conversionRealizada);
    Serial.println(analogico);
  }

 logicaRobot();
  
  if(estadoRobot != botones){
    pingControl(&distancia);
    lcdControl();
  }
}
