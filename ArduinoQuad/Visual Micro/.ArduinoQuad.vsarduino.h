/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Sanguino W/ ATmega644P or ATmega644PA (20 MHz), Platform=avr, Package=arduino
*/

#define __AVR_ATmega644p__
#define __AVR_ATmega644P__
#define ARDUINO 165
#define ARDUINO_MAIN
#define F_CPU 20000000L
#define __AVR__
#define F_CPU 20000000L
#define ARDUINO 165
#define ARDUINO_AVR_SANGUINO
#define ARDUINO_ARCH_AVR
extern "C" void __cxa_pure_virtual() {;}

void init_pid();
static void checkState();
void arm(uint8_t value);
void armingLoop();
//
//
void init_command();
void unrecognized();
void LED_on();
void LED_off();
void setP();
void setI();
void setD();
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
void init_imu();
void imu_caculate();
int16_t limit(int16_t value, int16_t low, int16_t high);
void changeThrottle(int throttle);
int readThrottle();
void writeTo4Escs(int throttle);
void init_esc();
void startUpMotors();
int normalizeThrottle(int value);
void pwmWrite(uint8_t chanel, uint16_t micro);
void init_receiver();
void thr_callback();
void rud_callback();
void ail_callback();
void ele_callback();
void rxRead();

#include "C:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\sanguino\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\arduino.h"
#include <ArduinoQuad.ino>
#include <CMD.ino>
#include <I2C.ino>
#include <IMU.ino>
#include <Kalman.h>
#include <MISC.ino>
#include <PID_v1.cpp>
#include <PID_v1.h>
#include <PWM.ino>
#include <PinChangeInt.h>
#include <Receiver.ino>
#include <SerialCommand.cpp>
#include <SerialCommand.h>
#include <global.h>
