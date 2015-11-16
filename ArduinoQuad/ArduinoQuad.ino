#include <Servo.h>
#include <Wire.h>
#include "PinChangeInt.h"
#include "Kalman.h"
#include "global.h"
#include "PID_v1.h"
#include "SerialCommand.h"


#define ROLL_P 0.848
#define ROLL_I 0.011
#define ROLL_D 0.661
#define ROLL_MAX_MOTOR_BALANCE_SPEED 7                  // max amount of thrust that will be applied to balance this axis
#define ROLL_PID_OUTPUT 50
#define ROLL_ERROR_CORRECTION 0

state_t State;
extern uint8_t RX_good;
extern double kalAngleX;
extern double kalAngleY;
extern int16_t RX[4];

SerialCommand command;

//PID area
double rollSp = 0;              // setpoints
double bal_roll= 0;             // motor balances can vary between -100 & 100, motor balance between axes -100:ac , +100:bd
double pitch, roll, yaw  = 0.0;	// angles in degrees
PID rollReg(&kalAngleX, &bal_roll, &rollSp, ROLL_P, ROLL_I, ROLL_D, DIRECT);

void init_pid()
{
	rollReg.SetMode(AUTOMATIC);
	rollReg.SetOutputLimits(-ROLL_PID_OUTPUT, ROLL_PID_OUTPUT);
	rollReg.SetSampleTime(14);
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
		LED = ON;
		rollReg.Reset();
		
	}
	else if (!value && State.Armed)
	{
		State.Armed = OFF;
		LED = OFF;
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

void setup() {
	Serial1.begin(115200);
	LED_DIR=1;
	init_esc();
	init_imu();
	init_receiver();
	init_pid();
	init_command();
	Serial1.println("init success");
}
uint16_t motor_pwm[5];
void loop()
{
	rxRead();
	checkState();
	armingLoop();
	imu_caculate();
	command.readSerial();
	
	rollSp=RX[AIL]/3;
	
	if(rollReg.Compute())
	{
		if (RX[THR] >25) RX[THR]=25;
		
		motor_pwm[2] = PWM_LOW + RX[THR]*2 + bal_roll;
		motor_pwm[4] = PWM_LOW + RX[THR]*2 - bal_roll;
		//Serial1.print(bal_roll*2); Serial1.print("\n");
	}
	
	if (State.Armed && !State.ThrottleOff)
	{
		pwmWrite(2,motor_pwm[2]);
		pwmWrite(4,motor_pwm[4]);
	}
	else
	{
		writeTo4Escs(0);
	}
	delay(2);
	
	#ifdef OUT_GRAPH
	Serial1.print(0); Serial1.print("\t");
	Serial1.print(0); Serial1.print("\t");
	Serial1.print(0); Serial1.print("\t");
	Serial1.print(kalAngleX); Serial1.print("\t");
	Serial1.print("\t");
	Serial1.print(0); Serial1.print("\t");
	Serial1.print(0); Serial1.print("\t");
	Serial1.print(0); Serial1.print("\t");
	Serial1.print(kalAngleY); Serial1.print("\t");
	Serial1.print("\r\n");
	#endif

}

