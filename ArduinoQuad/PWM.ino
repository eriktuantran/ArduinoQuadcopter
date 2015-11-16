/*
 * PWM.ino
 *
 * Created: 11/15/2015 9:50:07 AM
 *  Author: QuocTuanIT
 */ 
#include "Servo.h"
#include "global.h"

//Create the 4 esc objects
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

//Esc pins
#define escPin1 22
#define escPin2 20
#define escPin3 18
#define escPin4 19

int16_t limit(int16_t value, int16_t low, int16_t high)
{
	if (value < low) return low;
	else if (value > high) return high;
	else return value;
}

//Change throttle value
void changeThrottle(int throttle) {
	int currentThrottle = readThrottle();
	
	int step = 1;
	if(throttle < currentThrottle) {
		step = -1;
	}
	
	// Slowly move to the new throttle value
	while(currentThrottle != throttle) {
		writeTo4Escs(currentThrottle + step);
		
		currentThrottle = readThrottle();
		delay(throttleChangeDelay);
	}
}

//Read the throttle value
int readThrottle() {
	int throttle = esc1.read();
	
	Serial1.print("Current throttle is: ");
	Serial1.print(throttle);
	Serial1.println();
	
	return throttle;
}

//Change velocity of the 4 escs at the same time
void writeTo4Escs(int throttle) {
	esc1.write(throttle);
	esc2.write(throttle);
	esc3.write(throttle);
	esc4.write(throttle);
}

//Init escs
void init_esc() {
	esc1.attach(escPin1, minPulseRate, maxPulseRate);
	esc2.attach(escPin2, minPulseRate, maxPulseRate);
	esc3.attach(escPin3, minPulseRate, maxPulseRate);
	esc4.attach(escPin4, minPulseRate, maxPulseRate);
	
	//Init motors with 0 value
	writeTo4Escs(0);
}

//Start the motors
void startUpMotors() {
	writeTo4Escs(50);
}

// Ensure the throttle value is between 0 - 180
int normalizeThrottle(int value) {
	if(value < 0) {
		return 0;
		
		} else if(value > 180) {
		return 180;
	}
	
	return value;
}

void pwmWrite(uint8_t chanel, uint16_t micro)
{
	micro=limit(micro,minPulseRate,maxPulseRate);
	switch(chanel)
	{
		case 1: esc1.writeMicroseconds(micro); break;
		case 2: esc2.writeMicroseconds(micro); break;
		case 3: esc3.writeMicroseconds(micro); break;
		case 4: esc4.writeMicroseconds(micro); break;
	}
}