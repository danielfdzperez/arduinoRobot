#ifndef LCD_H_
#define LCD_H_

void sendCharacter(char character);
void sendString(char* string);
void setCursor(int f, int c);
void sendInteger(long int ln);
int configureLCD(void);
void clear(void);
void returnHome(void);
void powerOn(void);

#endif
