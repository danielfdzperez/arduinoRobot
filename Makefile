CC=avr-gcc
F_CPU=16000000L
MMCU=atmega32u4
P=m32u4
NOMBRE=lcd
PUERTO=/dev/ttyACM0
VELOCIDAD=57600
PROTOCOLO=avr109

all : $(NOMBRE).hex

$(NOMBRE).o : $(NOMBRE).c
	$(CC) -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c -o $@ $<

$(NOMBRE) : $(NOMBRE).o
	$(CC) -mmcu=$(MMCU) $(NOMBRE).o -o $(NOMBRE)
$(NOMBRE).hex : $(NOMBRE)
	avr-objcopy -O ihex -R .eeprom $(NOMBRE) $(NOMBRE).hex

subir : $(NOMBRE).hex
	avrdude -c $(PROTOCOLO) -p $(P) -P $(PUERTO) -D -b $(VELOCIDAD) -U flash:w:$(NOMBRE).hex
