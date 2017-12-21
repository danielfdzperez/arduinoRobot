#include <SoftwareSerial.h>
extern "C"{
  #include "common.h"
  #include "lcd.h"
}

#include<avr/io.h>

DatosEnviar datosInternosEnvio;
  
enum TlcdStatus {enviado, enviando, enviar,fin};
enum TmovimientoCursor {mover, moviendo, movido, quieto};
enum TmovimientoCursor movimientoCursor = quieto;
enum TlcdStatus lcdStatus = enviar;

int cifras = 3;
int distancia = 587;

void setup() {
  Serial.begin(57600);
  while (!Serial);
  configureLCD(&datosInternosEnvio);
  clear();
  returnHome();
  powerOn();

  setCursor(0,0);
  sendString("Prpcyon");
  setCursor(1,3);
  sendString("Puto amo");
  int N = 666;

  //sendInteger(N); // o bien TCNT1


  
  //Configurar el reloj
  OCR0A = 79;//10us
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A = (1<<1);// | (1<<COM0A1) | (1<<COM0A0); //Modo CTC
  TCCR0B = 1;//(1<<CS00);// | (1<<CS02);//Sin preescaler
  TIMSK0 = (1<<OCIE0A); 

}

ISR(TIMER0_COMPA_vect){
  datosInternosEnvio.tActual ++;
  //tPulsoEcho ++;
  //totalPing ++;
  //tPulsoPing ++;
  
}


void loop() {
  
  if(lcdStatus == enviar){
     distancia = sendNumberAsincrono(distancia,&datosInternosEnvio);
     lcdStatus = enviando;
     Serial.println("enviar");
  }
  
  if(lcdStatus == enviando && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    distancia = sendNumberAsincrono(distancia,&datosInternosEnvio);
    lcdStatus = enviado;
    Serial.println("enviando");
  }
  if(lcdStatus == enviado && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    lcdStatus = fin;
    cifras --;
    Serial.println("enviado");
  }

  if(movimientoCursor == mover){
    Serial.println("mover");
    movimientoCursor = moviendo;
    setCursorAsincrono(1, 7, &datosInternosEnvio);
  }
  if(movimientoCursor == moviendo && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    setCursorAsincrono(1, 7, &datosInternosEnvio);
    movimientoCursor = movido;
    Serial.println("moviendo");
  }
  if(movimientoCursor == movido && datosInternosEnvio.tActual >= datosInternosEnvio.espera/10){
    movimientoCursor = quieto;
    Serial.println("movido");
  }
  


  if(lcdStatus == fin){
    if(cifras)
      lcdStatus = enviar;
    else
      if(movimientoCursor == quieto)
         movimientoCursor = mover;
      
  }
}
