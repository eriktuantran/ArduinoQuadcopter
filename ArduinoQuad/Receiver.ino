/*
 * Receiver.ino
 *
 * Created: 11/15/2015 9:50:07 AM
 *  Author: QuocTuanIT
 */ 


int16_t RX[4];
uint16_t RX_raw[4];
uint8_t RX_good;
static int16_t RX_isr[4];
static uint8_t _mode;
static uint8_t _RX_good;

void init_receiver()
{
	PCintPort::attachInterrupt(RX_THR_PIN, thr_callback,CHANGE);
	PCintPort::attachInterrupt(RX_RUD_PIN, rud_callback,CHANGE);
	PCintPort::attachInterrupt(RX_AIL_PIN, ail_callback,CHANGE);
	PCintPort::attachInterrupt(RX_ELE_PIN, ele_callback,CHANGE);
}

void thr_callback()
{
	static uint16_t _start;
	uint16_t t = micros();
	sei();
	
	if (digitalRead(RX_THR_PIN))
	_start = t;
	else
	{
		RX_isr[THR] = t - _start;
		_RX_good |= _BV(THR);
	}
}
void rud_callback()
{
	static uint16_t _start;
	uint16_t t = micros();
	sei();
	
	if (digitalRead(RX_RUD_PIN))
	_start = t;
	else
	{
		RX_isr[RUD] = t - _start;
		_RX_good |= _BV(RUD);
	}
}

void ail_callback()
{
	static uint16_t _start;
	uint16_t t = micros();
	sei();
	
	if (digitalRead(RX_AIL_PIN))
	_start = t;
	else
	{
		RX_isr[AIL] = t - _start;
		_RX_good |= _BV(AIL);
	}
}
void ele_callback()
{
	static uint16_t _start;
	uint16_t t = micros();
	sei();
	
	if (digitalRead(RX_ELE_PIN))
	_start = t;
	else
	{
		RX_isr[ELE] = t - _start;
		_RX_good |= _BV(ELE);
	}
}
uint16_t RX_ZERO[]={PWM_RC_MID,PWM_RC_MID,PWM_RC_MID,PWM_RC_LOW};
void rxRead()
{
	uint16_t b;
	uint8_t index;
	
	EVERYMS(50)
	{
		RX_good = _RX_good;
		_RX_good = 0;
	}
	for (uint8_t i = 0; i < RX_CHANNELS; i++)
	{
		index = i;
		b = RX_isr[index];
		if (b >= 900 && b <= 2100)
		{
			RX_raw[i] = b;
			RX[i] = (int16_t)(RX_raw[i] - RX_ZERO[i]) >> 2;
		}
	}
	RX[THR]	>>= 1;
	if(RX[THR]<THR_CUTOFF) RX[THR]=0;
	else if (RX[THR]>100) RX[THR]=100;
	for (int i=0; i<3; i++)
	{
		RX[i]=limit(RX[i],-100,100);
		if (abs(RX[i]) <= 3 ) RX[i]=0;
	}
}
