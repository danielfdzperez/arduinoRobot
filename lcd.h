#ifndef LCD_H_
#define LCD_H_  
enum Parte { alta, baja };
typedef struct {
    enum Parte parte;
    unsigned int espera;
    unsigned int tActual;
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

#endif
