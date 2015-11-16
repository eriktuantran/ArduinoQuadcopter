/*
 * global.h
 *
 * Created: 11/15/2015 9:53:42 AM
 *  Author: QuocTuanIT
 */ 


#ifndef GLOBAL_H_
#define GLOBAL_H_

#define OUT_GRAPH
#define OUT_RX

#define ON		1
#define OFF		0
//Min and max pulse
#define minPulseRate        1000
#define maxPulseRate        2000
#define PWM_MIN				1000
#define PWM_LOW				1145
#define PWM_MAX				2000
#define throttleChangeDelay 50

#define WAIT_MS(ms) for(unsigned long xx = millis(); (millis() - xx) < ms;)
#define WAIT_US(us) for(unsigned long xx = micros(); (micros() - xx) < us;)
#define EVERYMS(ms) static uint16_t __CONCAT(_t,__LINE__); for(uint16_t _m = millis(); _m - __CONCAT(_t,__LINE__) >= ms; __CONCAT(_t,__LINE__) = _m)

#define RX_CHANNELS		4
#define PWM_RC_LOW		985
#define PWM_RC_MID		1404
#define PWM_RC_MAX		2100
#define THR_CUTOFF		3
#define RX_THR_PIN		21
#define RX_AIL_PIN		2
#define RX_ELE_PIN		0
#define RX_RUD_PIN		23
#define AIL				0
#define ELE				1
#define RUD				2
#define THR				3

typedef struct
{
	uint8_t Armed;
	uint8_t ThrottleOff;

	#define ERR_NO_ROLL			0x01
	#define ERR_NO_PITCH		0x02
	#define ERR_NO_YAW			0x04
	#define ERR_NO_THR			0x08
	#define ERR_NO_RX			(ERR_NO_ROLL | ERR_NO_PITCH | ERR_NO_YAW | ERR_NO_THR)
	uint8_t Error;
} state_t;

typedef struct
{
	unsigned int bit0 : 1;
	unsigned int bit1 : 1;
	unsigned int bit2 : 1;
	unsigned int bit3 : 1;
	unsigned int bit4 : 1;
	unsigned int bit5 : 1;
	unsigned int bit6 : 1;
	unsigned int bit7 : 1;
} volatile _bitreg8;

#define _REG_BIT2(r,b)	((*(_bitreg8*)&r).bit ## b)
#define _REG_BIT(r,b)	_REG_BIT2(r,b)

#define OUTPUT		1
#define INPUT		0
/// LED
#define LED_PORT	PORTB
#define LED_DDR		DDRB
#define LED_PIN		PINB
#define LED_BIT		3
#define LED			_REG_BIT(LED_PORT, LED_BIT)
#define LED_DIR		_REG_BIT(LED_DDR, LED_BIT)
#define LED_TOGGLE	(_REG_BIT(LED_PIN, LED_BIT) = 1)
#define LED_BLINK	(((LED_PORT>>LED_BIT)&0x1) == 0x1) ? (cbi(LED_PORT,LED_BIT)):(sbi(LED_PORT,LED_BIT))

#define RX_THRESHOLD		50		// was 75 [10/14/2015 QuocTuanIT]
#define ARM_DELAY			500	// in ms
#define THROTTLE_OFF	5

#endif /* GLOBAL_H_ */