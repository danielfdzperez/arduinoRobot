#include <SoftwareSerial.h>
extern "C"{
  #include "common.h"
  #include "lcd.h"
}

#include<avr/io.h>


/*Variables para controlar los estados del lcd*/
DatosEnviar datosInternosEnvio;
  
enum TlcdStatus {enviado, enviando, enviar, fin, preparado};
enum TmovimientoCursor {mover, moviendo, movido, quieto};
enum TmovimientoCursor movimientoCursor = quieto;
enum TlcdStatus lcdStatus = enviar;

int cifras = 3;
int distancia = 0;
int distanciaAnterior = 0;
int distanciaImprimir = 0;
/*-----------------------------------------------------------------*/

/*Variables de control de PING*/
volatile unsigned long tPulsoEcho = 0;
volatile int totalPing = 0;
volatile int tPulsoPing = 0;
enum TpingStatus {nada,recibir,recibido,respuesta,bloqueo};
volatile enum TpingStatus recibiendo = nada;
//volatile int recibiendo = 0;//0-> No se espera nada. 1-> Se espera recibir echo. 2-> Echo recibido. 3-> Gestionar respuesta. 4 -> Estado de bloqueo
volatile int tiempo = 0;
int pulso = 0;
unsigned char triggerPort = D9;
/*------------------------------------------------------------------*/

//Prepara el LCD
void prepararLCD(){
    configureLCD(&datosInternosEnvio);
  
  clear();
  returnHome();
  powerOn();
  
  
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

void setup() {
  Serial.begin(57600);
  while (!Serial);

  prepararLCD();
  configurarPing();

  configurarTimer0();
  


}

ISR(TIMER0_COMPA_vect){
  datosInternosEnvio.tActual ++;
  tPulsoEcho ++;
  totalPing ++;
  tPulsoPing ++;
}

//Captura interrupcion del echo
ISR(INT0_vect){
  if(recibiendo == recibir){
    //Recibe el echo
    tPulsoEcho = 0;
    recibiendo = recibido;
  }else
    if(recibiendo == recibido){
      //lo deja de recibir
      tiempo = tPulsoEcho; 
      recibiendo = respuesta;     
    }
      
}

void lcdControl(){
  if(lcdStatus == enviar){
     distanciaImprimir = sendNumberAsincrono(distanciaImprimir,&datosInternosEnvio);
     lcdStatus = enviando;
     //Serial.println("enviar");
  }
  
  if(lcdStatus == enviando && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    distanciaImprimir = sendNumberAsincrono(distanciaImprimir,&datosInternosEnvio);
    lcdStatus = enviado;
    //Serial.println("enviando");
  }
  if(lcdStatus == enviado && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    lcdStatus = fin;
    cifras --;
    //Serial.println("enviado");
  }

  if(movimientoCursor == mover){
    //Serial.println("mover");
    movimientoCursor = moviendo;
    setCursorAsincrono(1, 8, &datosInternosEnvio);
  }
  if(movimientoCursor == moviendo && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    setCursorAsincrono(1, 8, &datosInternosEnvio);
    movimientoCursor = movido;
    //Serial.println("moviendo");
  }
  if(movimientoCursor == movido && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    movimientoCursor = quieto;
    //Serial.println("movido");
  }
  


  if(lcdStatus == fin){
    if(cifras)
      lcdStatus = enviar;
    else{
      movimientoCursor = mover;
      lcdStatus = preparado;
      //Serial.println(lcdStatus);
    } 
  }

  if(lcdStatus == preparado && movimientoCursor == quieto && distanciaAnterior != distancia){
    cifras = 3;
    distanciaImprimir = distancia;
    distanciaAnterior = distancia;
    lcdStatus = enviar;
  }
}

void pingControl(int * distancia){
  
   if(!pulso && recibiendo==nada){
     
    //Comenzar a enviar la peticion de echo
    //PORTB |= (1<<5);
    writeHigh(triggerPort);
    pulso = 1;
    tPulsoPing = 0;
    totalPing = 0;
  }
  if(pulso && tPulsoPing >= 1){
   
    //Fin peticion echo
    writeLow(triggerPort);//PORTB &= ~(1<<5);
    pulso = 0;
    recibiendo = recibir;
    totalPing = 0;
  }
  if(recibiendo == respuesta){
     
    //Echo recibido
    recibiendo = bloqueo;  
    totalPing = 0;
    //Serial.println(tiempo/10);
    *distancia = tiempo/10;
    tiempo = 0;
    tPulsoEcho = 0;
  }
  if(totalPing >= 6000){
    //Fin tiempo de espera para el siguiente echo
    recibiendo = nada;
    totalPing = 0;
  }
}

void loop() {
  
  pingControl(&distancia);
  lcdControl();
}
