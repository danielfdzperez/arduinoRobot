CC=avr-gcc
F_CPU=16000000L
MMCU=atmega32u4
P=m32u4
NOMBRE=timer
PUERTO=/dev/ttyACM0
VELOCIDAD=57600
PROTOCOLO=avr109
LIBS=common.c lcd.c timer.c
OBJECTS=$(LIBS:.c=.o)


all : $(NOMBRE).hex
	make clean
#$(NOMBRE).o : $(NOMBRE).c
#	$(CC) -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c -o $@  $< 

.c.o:
	$(CC) -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c -o $@  $< 

$(NOMBRE) : $(NOMBRE).o $(OBJECTS)
	$(CC) -mmcu=$(MMCU) $(OBJECTS) -o $(NOMBRE)

$(NOMBRE).hex : $(NOMBRE)
	avr-objcopy -O ihex -R .eeprom $(NOMBRE) $(NOMBRE).hex

subir : $(NOMBRE).hex
	avrdude -c $(PROTOCOLO) -p $(P) -P $(PUERTO) -D -b $(VELOCIDAD) -U flash:w:$(NOMBRE).hex

clean : 
	-rm $(OBJECTS) *.hex
