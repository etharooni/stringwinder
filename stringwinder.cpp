//CNC String Winder v0.1 by Ethan Russell
//designed for atmel atmega328p cpu

#define F_CPU 16000000UL

#define maxrpm_s 100
#define maxrpm_c maxrpm_s

#define rho1 7725.0
#define d1 0.00045
#define rho2 8873.0
#define d2 0.00031
#define l 0.6
#define b 0.0
#define lambda10 (rho1*M_PI*d1*d1)/4.0
#define lambda20 (rho2*M_PI*d2*d2)/4.0
#define r (d1+d2)/2.0

#define prescale_s 64
#define stepsperrot_s 200
#define microstep_s 8
#define spindleStepPin PB3
#define spindleDirPin PB2
#define spindleDisablePin PD3

#define prescale_c 64
#define rotpermeter_c 200
#define stepsperrot_c 200
#define microstep_c 8
#define carriageStepPin PB4
#define carriageDirPin PB0
#define carriageDisablePin PB1

#define steppulse_us 350

#define limitFar PD5
#define limitClose PD2
#define rewindButton PD6
#define startButton PD7

#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>
#include <string.h>

long long steps_spindle = 0;
long long steps_carriage = 0;
bool carriageDirection = false;

double maxDensity;

double newVelocity(double meters, double angularVelocity){
	double dtheta_dx=(((maxDensity*(1.0-abs(b)))/(1.0+b*cos((2.0*M_PI*meters)/l)))-lambda10)*(1.0/lambda20);
	dtheta_dx = dtheta_dx*dtheta_dx-1.0;
	dtheta_dx = sqrt(dtheta_dx)/r;
	//char str[20];
	//snprintf(str,20,"%d,",(int)(dtheta_dx));
	//USART0String(str);
	
	if (dtheta_dx > 0.0){
		double velocity = angularVelocity/dtheta_dx;
		return velocity;
	}else{
		return 0.0;
	}
}

/*
     |  /|  /|
tcnt | / | / |
     |/| |/| |
comp B A B A B
step --__--__-
*/

ISR(TIMER0_COMPA_vect){
	PORTB &= ~(1 << spindleStepPin); //turn off step pin
	steps_spindle++;
}

ISR(TIMER1_COMPA_vect){
	PORTB &= ~(1 << carriageStepPin);
	steps_carriage++;
}

ISR(TIMER0_COMPB_vect){
	PORTB |= (1 << spindleStepPin); //turn on step pin
	TCNT0 = 0;
}

ISR(TIMER1_COMPB_vect){
	PORTB |= (1 << carriageStepPin);
	TCNT1 = 0;
}

double angV(double rpm){
	return (rpm*2.0*M_PI)/60.0;
}

void setSpindleRPM(double rpm){
	int compareValue = (int)(60.0*F_CPU/prescale_s/microstep_s/stepsperrot_s/rpm);
	OCR0B = compareValue;
}

void setCarriageRPM(double rpm){
	int compareValue = (int)(60.0*F_CPU/prescale_c/microstep_c/stepsperrot_c/rpm);
	OCR1B = compareValue;
}

void setCarriageVelocity(double metersPerSecond){
	//USART0Number((int)(metersPerSecond*100000.0));
	//USART0SendByte('\n');
	int compareValue = (int)((double)F_CPU/prescale_c/microstep_c/stepsperrot_c/rotpermeter_c/metersPerSecond);
	OCR1B = compareValue;
}

double getCarriageDistance(){
	return (double)steps_carriage/stepsperrot_c/microstep_c/rotpermeter_c;
}

void setCarriageDirection(bool direction){
	if(direction){
		PORTB |= (1 << carriageDirPin);
	}else{
		PORTB &= ~(1 << carriageDirPin);
	}
	carriageDirection = direction;
}


void USART0Init(void){
	// Set baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable transmission and reception
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
}
void USART0SendByte(uint8_t u8Data){
	//wait while previous byte is completed
	while(!(UCSR0A&(1<<UDRE0))){};
	// Transmit data
	UDR0 = u8Data;
}

uint8_t USART0ReceiveByte(){
	// Wait for byte to be received
	while(!(UCSR0A&(1<<RXC0))){};
	// Return received data
	return UDR0;
}

void USART0String(char* blah){
	int length = strlen(blah);
	for(int i=0; i<length; i++){
		USART0SendByte(blah[i]);
	}
}

void USART0Number(int number){
	char snum[5];

	// convert 123 to string [buf]
	itoa(number, snum, 10);
	USART0String(snum);
}


void setup(void){ 
	DDRB |= (1 << carriageStepPin)|(1 << spindleStepPin)|(1 << carriageDisablePin)|(1 << spindleDirPin)|(1 << carriageDirPin); // Set step pin as output
	DDRD = 0; //set all pins as inputs
	//PORTD = 0xFF;
	DDRD |= (1 << spindleDisablePin); //except for spindleDisable
	TCCR1B = 0b0011; //set the timer1 prescaler to 64 (bit 3 for CTC) <- p.137 of doc8161
	// 1 / ( (16000000/64) / (2^8) ) * 1000 = 1.024ms
	//         ^cpu   ^prescale ^bits = 976,562 hertz
	
	TCCR0B = 0b0011; //fcpu/64 - not a 16 bit timer - different prescale table
	
	TIMSK1 |= (1<<OCIE1A)|(1<<OCIE1B); //enable compA, compB for timer2
	TIMSK0 |= (1 << OCIE0A)|(1<<OCIE0B); //compA, compB timer 0
	
	TCNT1 = 0; //reset and init counter
	TCNT0 = 0;
	
	int spindleStepCompare = (int)ceil(((double)F_CPU*steppulse_us)/(1000000.0*prescale_s)); //how long should we keep the step pin on every pulse?
	OCR0A = spindleStepCompare;
	
	int carriageStepCompare = (int)ceil(((double)F_CPU*steppulse_us)/(1000000.0*prescale_c));
	OCR1A = carriageStepCompare;
	
	PORTB |= (1 << carriageDisablePin); //disable on start up
	PORTD |= (1 << spindleDisablePin);
	
	double maxDensity = (2.0*M_PI*r/d2);
	maxDensity = lambda10+lambda20*sqrt(1.0+maxDensity*maxDensity);
	
	setCarriageDirection(0);
	USART0Init();
	
	//sei(); //  Enable global interrupts
	//F_CPU/prescale/OCR1A/stepsperrot/microstep/rotpermeter  = meter/sec
	//   steps/sec            |    rot/sec  |  meter/sec
}

void disableAll(){
	cli();
	PORTB |= (1 << carriageDisablePin);
	PORTD |= (1 << spindleDisablePin);
}

bool getFarLimit(){
	int buttons = PIND;
	bool outsideLimit = !(buttons & (1 << limitFar))==0;
	bool closeLimit = !(buttons & (1 << limitClose))==0;
	//bool blackButton = (buttons & (1 << rewindButton))==0;
	//bool redButton = (buttons & (1 << startButton))==0;
	if(carriageDirection){
		return closeLimit;
	}else{
		return outsideLimit;
	}

}

bool getRedButton(){
	int buttons = PIND;
	//bool outsideLimit = !(buttons & (1 << limitFar))==0;
	//bool closeLimit = !(buttons & (1 << limitClose))==0;
	//bool blackButton = (buttons & (1 << rewindButton))==0;
	bool redButton = (buttons & (1 << startButton))==0;
	return redButton;
}

bool getBlackButton(){
	int buttons = PIND;
	//bool outsideLimit = !(buttons & (1 << limitFar))==0;
	//bool closeLimit = !(buttons & (1 << limitClose))==0;
	bool blackButton = (buttons & (1 << rewindButton))==0;
	//bool redButton = (buttons & (1 << startButton))==0;
	return blackButton;
}

void rewindCarriage(){
	sei();
	PORTD |= (1 << spindleDisablePin); //disable spindle
	PORTB &= ~(1 << carriageDisablePin);
	
	setCarriageDirection(1);

	//setCarriageRPM(maxrpm_c);
	setCarriageVelocity(0.005);
	while(!getFarLimit() && !getRedButton()){
		_delay_ms(1);
	}
	disableAll();
}

void windString(){
	steps_carriage = 0;
	steps_spindle = 0;
	
	double dtheta_dt = angV(maxrpm_s);
	setSpindleRPM(maxrpm_s);
	double meters = 0.0;
	double velocity = 0.0;
	
	sei();
	
	setCarriageDirection(0);
	
	PORTB &= ~(1 << carriageDisablePin); //enable the steppers
	PORTD &= ~(1 << spindleDisablePin);
	
	while(meters < l && !getFarLimit() && !getBlackButton()){
		meters = getCarriageDistance();
		velocity = newVelocity(meters, dtheta_dt);
		setCarriageVelocity(velocity);
		_delay_ms(5);
	}
	disableAll();
}

int main (void){
	setup();
	disableAll();
	USART0String("hello!\n");
	while(1){
		_delay_ms(25);
		if (getBlackButton()){
			//USART0String("rewinding.   ");
			rewindCarriage();
			_delay_ms(500);
		}else if (getRedButton()){
			//USART0String("winding.   ");
			windString();
			_delay_ms(500);
		}
	}
}