avr-gcc -g -Os -mmcu=atmega328p -c stringwinder.cpp
avr-gcc -g -mmcu=atmega328p -o stringwinder.elf stringwinder.o
avr-objcopy -O ihex stringwinder.elf stringwinder.hex