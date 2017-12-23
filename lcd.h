#ifndef LCD_H_
#define LCD_H_  
enum Parte { alta, baja };
/*
 * Estructura se utiliza en las funciones asincronas.
 * tActual debe incrementarse en un timer
 * */
typedef struct {
    enum Parte parte;//Parte que le toca enviar al LCD de un dato
    unsigned int espera;//Cuanto debe esperar para la siguiente accion
    unsigned int tActual;//Tiempo actual 
}volatile DatosEnviar;

void sendCharacter(char character);
int sendNumberAsincrono(unsigned int, DatosEnviar *);
int setCursorAsincrono(int , int , DatosEnviar *);
void sendString(char* string);
void setCursor(int f, int c);
void sendInteger(long int ln);
int configureLCD(DatosEnviar *);
void clear(void);
void returnHome(void);
void powerOn(void);
void escribirIzq(void);

#endif
