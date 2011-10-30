#include "project/robotathon_drivers/root2.h"
#include "project/robotathon_drivers/linesensor.h"
#include "project/robotathon_drivers/encoder.h"
#include "project/robotathon_drivers/IR.h"
#include "project/robotathon_drivers/motor.h"
#include "project/robotathon_drivers/servo.h"
#include "project/robotathon_drivers/usb.h"

#include "sysinit.h"

// Set pin 2.10 (onboard LED) as an output
void initLED() {
	gpioSetDir(2, 10, 1);
}

// Call all of the necessary initialization functions
void root2Init() {
	// Configure cpu, usbcdc, and mandatory peripherals
	systemInit();

	// Configure on-board LED and servo, linesensor, IR drivers, etc.
	initLED();
	initServos();
	initLineSensor();
	initIR();
	initMotors(0,0);
	initEncoders(1,1);
}

// Pause program for [ms] milliseconds
void pause(int ms) {
	systickDelay(ms);
}

// Toggle pin 2.10 (onboard LED) 
void toggleLED() {
	int current_value = gpioGetValue(2, 10);
	if (current_value == 0)
		gpioSetValue(2, 10, 1);
	else 
		gpioSetValue(2, 10, 0);
}