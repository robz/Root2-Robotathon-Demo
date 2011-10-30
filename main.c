/**************************************************************************/
/*! 
    @file     main.c
    @author   Robby Nevels, Cruz Monrreal, Stephen Hall, Frank Weng, Razik Ahmed
*/
/**************************************************************************/

#include <stdio.h>

#define CFG_USBCDC

#include "projectconfig.h"
#include "sysinit.h"

#include "project/robotathon_drivers/linesensor.h"
#include "project/robotathon_drivers/encoder.h"
#include "project/robotathon_drivers/IR.h"
#include "project/robotathon_drivers/motor.h"
#include "project/robotathon_drivers/servo.h"
#include "project/robotathon_drivers/usb.h"
#include "project/robotathon_drivers/root2.h"

void motorDemo();
void servoDemo();
void IRDemo();
void lineSensorDemo();
void encoderDemo();

/**************************************************************************/
/*
Begin: main
*/
/**************************************************************************/

int main()
{
	// Define menu string
	char intro[] = "\n\r** Welcome to the Robotathon Root2 Demo! **\n\r\
Choose an option:\n\r1 - Motor Demo\n\r2 - Servo Demo\n\r\
3 - IR Demo\n\r4 - Line Sensor Demo\n\r5 - Encoder Demo\n\r";

	// Initialize system
	root2Init();

	// Wait the someone to connect and press a key
	waitForKey();

	// Loop forever waiting for someone to request a demo
	while (1) {
		printf("%s>>", intro);
		char choice = getKey();
		printf("%c\n\r", choice);
		switch(choice) {
			case '1' : motorDemo(); break;
			case '2' : servoDemo(); break;
			case '3' : IRDemo(); break;
			case '4' : lineSensorDemo(); break;
			case '5' : encoderDemo(); break;
			default : printf("invalid choice!\n\r"); 
			break; 
		}
		toggleLED();
    }

	return 0;
}

/**************************************************************************/
/*
Begin: Demo functions
*/
/**************************************************************************/

void servoDemo(void) {
	int degrees[] = {0, 0, 0};
	int servoNum = 1, quit = 0;

	printf("\n\rWelcome to the servo demo!\n\r\
Choose an option:\n\r# - change servo (default is 1)\n\r\
a - left\n\rd - right\n\rq - quit\n\r");

	while (quit != 1) {
		printf("Servo %d >>", servoNum);
		char choice = getKey();
		printf("%c\n\r", choice);
		switch(choice) {
			case 'a' : moveServo(servoNum, degrees[servoNum-1]+=5); break;
			case 'd' : moveServo(servoNum, degrees[servoNum-1]-=5); break;
			case '1' : servoNum = 1; break;
			case '2' : servoNum = 2; break;
			case '3' : servoNum = 3; break;
			case 'q' : quit = 1; break;
			default : printf("invalid choice!\n\r");
			break; 
		}
		printf("%d %d %d\n\r", degrees[0], degrees[1], degrees[2]);
		toggleLED();
    }
}

int duties[] = {50, 50};

void motorDemo() {
	int motorNum = 0, quit = 0;

	printf("\n\rWelcome to the motor demo!\n\r\
Choose an option:\n\rspace - stops motors\n\r\
# - change motor (default is 1)\n\ra - increase duty cycle\n\r\
d - decrease duty cycle\n\rq - quit\n\r");

	while (quit != 1) {
		printf("Motor %d >>", motorNum+1);
		char choice = getKey();
		printf("%c\n\r", choice);
		switch(choice) {
			case 'a' : 
				duties[motorNum]+=1;
				if (duties[motorNum] > 100) duties[motorNum] = 100;
				setMotorDuty(motorNum, duties[motorNum]); 
				break;
			case 'd' : 
				duties[motorNum]-=1;
				if (duties[motorNum] < 0) duties[motorNum] = 0;
				setMotorDuty(motorNum, duties[motorNum]); 
				break;
			case '1' : motorNum = 0; break;
			case '2' : motorNum = 1; break;
			case ' ' : 
				duties[0] = duties[1] = 50;
				setMotorDuty(0, duties[0]); 
				setMotorDuty(1, duties[1]); 
				break;
			case 'q' : quit = 1; break;
			default : printf("invalid choice!\n\r"); 
			break; 
		}
		printf("%d %d\n\r", duties[0], duties[1]);
		toggleLED();
    }
}

void IRDemo() {
	int count = 0;

	printf("\n\r** Welcome to the IR demo! **\n\r\
Choose an option:\n\rspace - start reading\n\rq - quit\n\r");
	
	char choice = checkKey();
	while (choice != 'q') 
	{
		printf("%03d : %04d  %04d  %04d  %04d\r", count, 
			getIRValue(0), getIRValue(1), getIRValue(2), getIRValue(3)); 
			
		count++;
		toggleLED();
		pause(100);
		choice = checkKey();
    }
	
	printf("\n");
}

void lineSensorDemo() {
	int lineSensorArr[8];
	int count = 0;

	printf("\n\r** Welcome to the line sensor demo! **\n\r\
Choose an option:\n\rspace - start reading\n\rq - quit\n\r");
	
	waitForKey();

	char choice = checkKey();
	while (choice != 'q') 
	{
		readLineSensor(lineSensorArr); 
		
		printf("%03d : %03d  %03d  %03d  %03d  %03d  %03d  %03d  %03d\r", 
			count, lineSensorArr[0], lineSensorArr[1], 
			lineSensorArr[2], lineSensorArr[3], lineSensorArr[4], 
			lineSensorArr[5], lineSensorArr[6], lineSensorArr[7]);
		/* 
		 * Display the line sensor data in this order for the old (purple) sensor:
		printf("%03d : %03d  %03d  %03d  %03d  %03d  %03d  %03d  %03d\r", 
			count, lineSensorArr[0], lineSensorArr[4], 
			lineSensorArr[1], lineSensorArr[5], lineSensorArr[2], 
			lineSensorArr[6], lineSensorArr[3], lineSensorArr[7]);
		*/	
		count++;
		toggleLED();
		pause(100);
		choice = checkKey();
    }
	
	printf("\n");
}

void encoderDemo() {
	int count = 0;

	printf("\n\r** Welcome to the encoder demo! **\n\r\
Choose an option:\n\rspace - start reading\n\rq - quit\n\r");
	
	waitForKey();
	
	char choice = checkKey();
	while (choice != 'q') 
	{
		int enc1_val = getEncoderValue(ENCODER_0),
			enc2_val = getEncoderValue(ENCODER_1);
		printf("%03d : %03d  %03d\r", 
			count, enc1_val, enc2_val);
			
		count++;
		toggleLED();
		pause(100);
		choice = checkKey();
    }
	
	printf("\n");
}