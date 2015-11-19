/*
* CMD.ino
*
* Created: 11/15/2015 9:50:07 AM
*  Author: QuocTuanIT
*/


void init_command()
{
	command.addCommand("on",LED_on);
	command.addCommand("off",LED_off);
	command.addCommand("rst",soft_reset);
	command.addCommand("arm",arm_control);
	command.addCommand("thr",thr_control);
	command.addCommand("imu",imu_angle);
	command.addCommand("max",setMax);
	command.addCommand("p",setP);
	command.addCommand("i",setI);
	command.addCommand("d",setD);
	command.addDefaultHandler(unrecognized);
}
void unrecognized()
{
	Serial1.println("WRONG CMD!!");
}
void LED_on()
{
	Serial1.println("LED on");
	digitalWrite(LED_PIN,HIGH);
}
void LED_off()
{
	Serial1.println("LED off");
	digitalWrite(LED_PIN,LOW);
}
void soft_reset()
{
	do
	{
		wdt_enable(WDTO_15MS);
		for(;;)
		{
		}
	} while(0);
}
void arm_control()
{
	int param;
	Serial1.print("ARM: ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atoi(arg);
		if (param) { arm(ON); Serial1.println("ON");}
		else { arm(OFF); Serial1.println("OFF");}
	}
	else Serial1.println("??? param");
}
void thr_control()
{
	int param;
	Serial1.print("THR: ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atoi(arg);
		RX[THR] = param;
		Serial1.println(param);
	}
	else
	{
		Serial1.println(RX[THR]);
	}
}
void imu_angle()
{
	Serial1.print("ROLL: "); Serial1.println(kalAngleX);
	Serial1.print("PITCH: "); Serial1.println(kalAngleY);
}

void setMax()
{
	float param;
	Serial1.print("Set Out Max ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atof(arg);
		Serial1.println(param);
		rollReg.SetOutputLimits(-param, param);
	}
	else Serial1.println("??? param");
}
void setP()
{
	float param;
	Serial1.print("Set P ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atof(arg);
		Serial1.println(param,4);
		//rollReg.SetTunings(param,rollReg.GetKi(),rollReg.GetKd());
		//pidSetP(param*SCALING_FACTOR,&pidData);
		PID_other_setP(param);
	}
	else Serial1.println("??? param");
}
void setI()
{
	float param;
	Serial1.print("Set I ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atof(arg);
		Serial1.println(param,4);
		//rollReg.SetTunings(rollReg.GetKp(),param,rollReg.GetKd());
		//pidSetI(param*SCALING_FACTOR,&pidData);
		PID_other_setI(param);
	}
	else Serial1.println("??? param");
}
void setD()
{
	float param;
	Serial1.print("Set D ");
	char *arg = command.next();
	if (arg != NULL)
	{
		param=atof(arg);
		Serial1.println(param,4);
		//rollReg.SetTunings(rollReg.GetKp(),rollReg.GetKi(),param);
		//pidSetD(param*SCALING_FACTOR,&pidData);
		PID_other_setD(param);
	}
	else Serial1.println("??? param");
}