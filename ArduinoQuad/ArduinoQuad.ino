#include <Servo.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "global.h"
#include "misc.h"
#include "PinChangeInt.h"
#include "Kalman.h"
#include "PID_v1.h"
#include "SerialCommand.h"

extern uint8_t RX_good;
extern double kalAngleX;
extern double kalAngleY;
extern int16_t RX[4];

state_t State;
SerialCommand command;
struct PID_DATA pidData;

//PID area
#define ROLL_P 0.2
#define ROLL_I 0.0
#define ROLL_D 0.0
#define ROLL_MAX_MOTOR_BALANCE_SPEED 7                  // max amount of thrust that will be applied to balance this axis
#define ROLL_PID_OUTPUT 50
#define ROLL_ERROR_CORRECTION 0

#define PITCH_P 0.848
#define PITCH_I 0.011
#define PITCH_D 0.661
#define PITCH_MAX_MOTOR_BALANCE_SPEED 7                  // max amount of thrust that will be applied to balance this axis
#define PITCH_PID_OUTPUT 50
#define PITCH_ERROR_CORRECTION 0

double rollSp, pitchSp = 0;            // setpoint
double rollOut, pitchOut = 0;        // PID OUTPUT
PID rollReg(&kalAngleX, &rollOut, &rollSp, ROLL_P, ROLL_I, ROLL_D, DIRECT);
PID pitchReg(&kalAngleY, &pitchOut, &pitchSp, PITCH_P, PITCH_I, PITCH_D, DIRECT);

void init_pid()
{
	rollReg.SetMode(AUTOMATIC);
	rollReg.SetOutputLimits(-ROLL_PID_OUTPUT, ROLL_PID_OUTPUT);
	rollReg.SetSampleTime(14);
	
	pitchReg.SetMode(AUTOMATIC);
	pitchReg.SetOutputLimits(-PITCH_PID_OUTPUT, PITCH_PID_OUTPUT);
	pitchReg.SetSampleTime(14);
}
static void checkState()
{
	State.ThrottleOff = RX[THR] < THROTTLE_OFF;
	uint8_t e = 0;
	e |= (~RX_good) & (ERR_NO_PITCH|ERR_NO_ROLL|ERR_NO_THR|ERR_NO_YAW);
	
	State.Error = e;
}

void arm(uint8_t value)
{
	if (value && !State.Armed)
	{
		State.Armed = ON;
		digitalWrite(LED_PIN,HIGH);
		//	pid_Reset_Integrator()
		//rollReg.Reset();
		reset_pid_other();
		
	}
	else if (!value && State.Armed)
	{
		State.Armed = OFF;
		digitalWrite(LED_PIN,LOW);
	}
}

void armingLoop()
{
	static uint16_t startArm;
	static uint16_t startOff;
	uint16_t t = millis();
	
	if (State.Error != 0) return;
	
	if (startArm == 0)
	{
		if (State.ThrottleOff && abs(RX[RUD]) > RX_THRESHOLD)
		startArm = t;
	}
	else if (!State.ThrottleOff || abs(RX[RUD]) < RX_THRESHOLD)
	startArm = 0;
	else if (t - startArm >= ARM_DELAY)
	{
		if (RX[RUD] > RX_THRESHOLD)
		arm(ON);
		else
		arm(OFF);
		startArm = 0;
	}
}


#define K_P     0.50
#define K_I     0.00
#define K_D     0.00


void setup()
{
	MCUSR = 0;
	wdt_disable();
	Serial.begin(115200);
	Serial1.begin(9600);
	pinMode(LED_PIN,OUTPUT);digitalWrite(BUZZ_PIN,LOW);
	pinMode(BUZZ_PIN,OUTPUT); digitalWrite(BUZZ_PIN,HIGH);
	init_esc();
	init_imu();
	init_receiver();
	//init_pid();
	init_command();
	//pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);
	Serial1.println("Init success");
}
uint16_t motor_pwm[5];
uint16_t offset = 30;
int16_t pid_roll_out=0;
extern float pid_output_roll;
extern float pid_output_pitch;
extern float pid_output_yaw;
extern float pid_roll_setpoint;
void loop()
{
	rxRead();
	checkState();
	armingLoop();
	imu_caculate();
	command.readSerial();
	
	pid_roll_setpoint=RX[AIL]/5;
	
	EVERYMS(20)
	{
		calculate_pid();
		if (RX[THR] > 30) RX[THR] = 30;
		motor_pwm[3] = PWM_LOW + offset + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
		motor_pwm[4] = PWM_LOW + offset - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
		motor_pwm[1] = PWM_LOW + offset - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
		motor_pwm[2] = PWM_LOW + offset + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
		
		 if (motor_pwm[1] < PWM_LOW) motor_pwm[1] = PWM_LOW;                                         //Keep the motors running.
		 if (motor_pwm[2] < PWM_LOW) motor_pwm[2] = PWM_LOW;                                         //Keep the motors running.
		 if (motor_pwm[3] < PWM_LOW) motor_pwm[3] = PWM_LOW;                                         //Keep the motors running.
		 if (motor_pwm[4] < PWM_LOW) motor_pwm[4] = PWM_LOW;                                         //Keep the motors running.
		 
		 if(motor_pwm[1] > PWM_MID)motor_pwm[1] = PWM_MID ;                                          //Limit the esc-1 pulse to 2000us.
		 if(motor_pwm[2] > PWM_MID)motor_pwm[2] = PWM_MID;                                           //Limit the esc-2 pulse to 2000us.
		 if(motor_pwm[3] > PWM_MID)motor_pwm[3] = PWM_MID;                                           //Limit the esc-3 pulse to 2000us.
		 if(motor_pwm[4] > PWM_MID)motor_pwm[4] = PWM_MID;                                           //Limit the esc-4 pulse to 2000us.
	}
	
	//EVERYMS(20)
	//{
	//RX[THR] = 10;
	//
	//pid_roll_out = pid_Controller(rollSp, kalAngleX, &pidData);
	//motor_pwm[1] = PWM_LOW + offset + pid_roll_out;
	//motor_pwm[2] = PWM_LOW + offset + pid_roll_out;
	//motor_pwm[3] = PWM_LOW + offset - pid_roll_out;
	//motor_pwm[4] = PWM_LOW + offset - pid_roll_out;
	//
	//}
	//if(rollReg.Compute())
	//{
	////	if (RX[THR] >40) RX[THR]=40;
	//RX[THR]=40;
	//motor_pwm[1] = PWM_LOW + offset + rollOut;
	//motor_pwm[2] = PWM_LOW + offset + rollOut;
	//motor_pwm[3] = PWM_LOW + offset - rollOut;
	//motor_pwm[4] = PWM_LOW + offset - rollOut;
	//}
	//if(pitchReg.Compute())
	//{
	//if (RX[THR] >25) RX[THR]=25;
	
	//motor_pwm[2] = PWM_LOW + RX[THR]*2 + pitchOut;
	//motor_pwm[4] = PWM_LOW + RX[THR]*2 - pitchOut;
	//Serial1.print(bal_roll*2); Serial1.print("\n");
	//}
	
	if (State.Armed && !State.ThrottleOff)
	{
		pwmWrite(1,motor_pwm[1]);
		pwmWrite(2,motor_pwm[2]);
		pwmWrite(3,motor_pwm[3]);
		pwmWrite(4,motor_pwm[4]);
	}
	else
	{
		writeTo4Escs(0);
	}
	delay(2);
	
	#ifdef OUT_MOTOR
	EVERYMS(100)
	{
		for (int i=0; i<4; i++)
		{
			Serial1.print(motor_pwm[i+1]); Serial1.print("\t");
			
		}
		Serial1.print("\n");
	}
	#endif
	
	#ifdef OUT_RX
	EVERYMS(100)
	{
		for (int i=0; i<4; i++)
		{
			Serial1.print(RX[i]); Serial1.print("\t");
			
		}
		Serial1.print("\n");
	}
	#endif
	
	#ifdef OUT_GRAPH
	Serial.print(0); Serial.print("\t");
	Serial.print(0); Serial.print("\t");
	Serial.print(0); Serial.print("\t");
	Serial.print(kalAngleX); Serial.print("\t");
	Serial.print("\t");
	Serial.print(0); Serial.print("\t");
	Serial.print(0); Serial.print("\t");
	Serial.print(0); Serial.print("\t");
	Serial.print(kalAngleY); Serial.print("\t");
	Serial.print("\r\n");
	#endif

}

