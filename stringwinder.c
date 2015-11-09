//CNC String Winder v0.1 by Ethan Russell
//designed for atmel atmega328p cpu

#define F_CPU 16000000UL

#define maxrpm 200

#define spindleStepPin PB1
#define spindleDirPin PB4
#define prescale_s 256
#define stepsperrot_s 200
#define microstep_s 1

#define carriageStepPin PB0
#define carriageDirPin PB3
#define prescale_c 64
#define rotpermeter_c 200
#define stepsperrot_c 200
#define microstep_c 1

#define disablePin PB2

#define steppulse_us 20

#define lambda10 0.00154
#define lambda20 0.00175
#define maxDensity 0.0127
#define b 0.25
#define l 1.0
#define r 0.0005

#include <avr/io.h> 
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>

long long steps_spindle = 0;
long long steps_carriage = 0;

double newVelocity(double meters, double angularVelocity){
	double dtheta_dx=(((maxDensity*(1-b))/(1+b*cos((2*M_PI*meters)/l)))-lambda10)*(1/lambda20);
	dtheta_dx = dtheta_dx*dtheta_dx-1;
	dtheta_dx = sqrt(dtheta_dx)/r;
	if (dtheta_dx > 0){
		double velocity = angularVelocity/dtheta_dx;
		return velocity;
	}else{
		return 0.0;
	}
}

double angV(double rpm){
	return (rpm*2*M_PI)/60;
}

int setmaxrpm(double rpm){
	int compareValue = (int)(60.0*F_CPU/prescale_s/microstep_s/stepsperrot_s/rpm);
	OCR0B = compareValue;
}

int setCarriageVelocity(double metersPerSecond){
	int compareValue = (int)(F_CPU/prescale_c/microstep_c/stepsperrot_c/rotpermeter_c/metersPerSecond);
	OCR1B = compareValue;
}

int setCarriageRPM(double rpm){
	int compareValue = (int)(60.0*F_CPU/prescale_c/microstep_c/stepsperrot_c/rpm);
	OCR1B = compareValue;
}

double getCarriageDistance(){
	return steps_carriage/stepsperrot_c/microstep_c/rotpermeter_c;
}

int rewindCarriage(){
	PORTB |= (1 << carriageDirPin); //flip the direction
	setCarriageRPM(maxrpm);
	while(getCarriageDistance() > 0.0){
		_delay_ms(1);
	}
}

/*
        /+  /+
       / | / |
      /  |/  |
     B A B A B
	 --__--__-
	 
*/

ISR(TIMER0_COMPA_vect){
	PORTB &= ~(1 << spindleStepPin);
	steps_spindle++;
}

ISR(TIMER1_COMPA_vect){
	PORTB &= ~(1 << carriageStepPin);
	steps_carriage++;
}

ISR(TIMER0_COMPB_vect){
	PORTB |= (1 << spindleStepPin);
	TCNT0 = 0;
}

ISR(TIMER1_COMPB_vect){
	PORTB |= (1 << carriageStepPin);
	TCNT1 = 0;
}

int setup(void){ 
	DDRB |= (1 << carriageStepPin)|(1 << spindleStepPin)|(1 << disablePin)|(1 << spindleDirPin)|(1 << carriageDirPin); // Set step pin as output
	TCCR1B = 0b0011; //set the timer1 prescaler to 64 (bit 3 for CTC) <- p.137 of doc8161
	// 1 / ( (16000000/64) / (2^8) ) * 1000 = 1.024ms
	//         ^cpu   ^prescale ^bits = 976,562 hertz
	
	TCCR0B = 0b0011; //fcpu/256 - not a 16 bit timer - different prescale table
	
	TIMSK1 |= (1<<OCIE1A)|(1<<OCIE1B); //enable compA, compB for timer2
	TIMSK0 |= (1 << OCIE0A)|(1<<OCIE0B); //compA, compB timer 0
	
	TCNT1 = 0; //reset and init counter
	TCNT0 = 0;
	
	int spindleStepCompare = (int)ceil((F_CPU*steppulse_us)/(1000000.0*prescale_s)); //how long should we keep the step pin on every pulse?
	OCR0A = spindleStepCompare;
	
	int carriageStepCompare = (int)ceil((F_CPU*steppulse_us)/(1000000.0*prescale_c));
	OCR1A = carriageStepCompare;
	
	PORTB &= ~(1 << carriageDirPin); //set initial direction
	PORTB &= ~(1 << spindleDirPin);
	
	PORTB &= ~(1 << disablePin);
	sei(); //  Enable global interrupts
	//F_CPU/prescale/OCR1A/stepsperrot/microstep/rotpermeter  = meter/sec
	//   steps/sec            |    rot/sec  |  meter/sec
}

int main (void){
	setup();
	double dtheta_dt = angV(maxrpm);
	setmaxrpm(maxrpm);
	double meters = 0.0;
	double velocity = 0.0;
	while(meters < l){
		meters = getCarriageDistance();
		velocity = newVelocity(meters, dtheta_dt);
		setCarriageVelocity(velocity);
		_delay_ms(1);
	}
	PORTB = 0;
	cli();
	PORTB |= (1 << disablePin);
}