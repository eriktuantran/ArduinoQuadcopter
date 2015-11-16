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
	digitalWrite(3,HIGH);
}
void LED_off()
{
	Serial1.println("LED off");
	digitalWrite(3,LOW);
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
		rollReg.SetTunings(param,rollReg.GetKi(),rollReg.GetKd());
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
		rollReg.SetTunings(rollReg.GetKp(),param,rollReg.GetKd());
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
		rollReg.SetTunings(rollReg.GetKp(),rollReg.GetKi(),param);
	}
	else Serial1.println("??? param");
}