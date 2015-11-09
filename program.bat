avr-g++ -g -Os -mmcu=atmega328 -c stringwinder.c
avr-gcc -g -mmcu=atmega328 -o stringwinder.elf stringwinder.o
avr-objcopy -O ihex stringwinder.elf stringwinder.hex