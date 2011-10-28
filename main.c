/**************************************************************************/
/*! 
    @file     main.c
    @author   Robby Nevels, Cruz Monrreal, Stephen Hall, Frank Weng, Razik Ahmed
*/
/**************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define CFG_USBCDC

#include "projectconfig.h"
#include "sysinit.h"

#include "core/timer32/timer32.h"
#include "core/usbcdc/cdcuser.h"
#include "core/i2c/i2c.h"
#include "core/adc/adc.h"



typedef enum { ENCODER_0, ENCODER_1 } encoder_t;	// encoder data type
typedef signed long encoder_count_t;				// encoder count type
typedef enum { s_0, s_1, s_2, s_3 }  encoder_state_t;
const int enc_Machine[4][4] = { { 0 , 1 , -1 , 0 } , {-1 , 0 , 0 , 1 } , { 1 , 0 , 0 , -1 } , { 0 , -1 , 1 , 0 } }; 
volatile static encoder_count_t enc_0, enc_1;
volatile static encoder_state_t enc_state[2];

int motor1_reversed = 0, motor2_reversed = 0, encoder1_reversed = 0, 
	encoder2_reversed = 0;
int duties[] = {50, 50};

char prompt[] = ">> ";

void root2Init();
void pause(int ms);
void toggleLED();
int getString(char* buf, int len);
void waitForKey();
char getKey();
void moveServo(int servo, int degree);
int readLineSensor(int LineSensor[8]);
int getIRValue(int channel);
void setMotorDuty(int motor, int power);
signed long getEncoderValue(encoder_t enc);

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
		printf("%s", intro);
		printf("%s", prompt);
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
End: main
Begin: Low-level functions and definitions
*/
/**************************************************************************/

//leftshifting 1 for space for R/W bit
#define ADS7830_ADDR (0x48 << 1)
#define READ_BIT 1

#define UARTBUFFERSIZE 5

/* begin encoder code */
/* Note: will need to add support for directions (software reverse) later. Same for motor code.*/
/* Move typedefs into header when sorting code 
 * Also recommending same typedef style for all other aspects of this project */

void encoderDrive( encoder_t enc )
{
	int nextState;
	if (enc == ENCODER_0)
	{
		nextState = (((gpioGetValue(3,0) != 0) << 1 ) + (gpioGetValue(3,1) != 0));
		enc_0 += enc_Machine[enc_state[0]][ nextState ];
		enc_state[0] = nextState;
	} 
	else if (enc == ENCODER_1)
	{
		nextState = (((gpioGetValue(3,2) != 0) << 1 ) + (gpioGetValue(3,3) != 0));
		enc_1 += enc_Machine[enc_state[1]][ nextState ];
		enc_state[1] = nextState;
	} 
}

/* called in PIOINT3_IRQHandler */
void encoderIntHandler(void){
	//printf("interrupt fire!");
  if ( gpioIntStatus(3, 0) ){
	  encoderDrive( ENCODER_0 );
	  gpioIntClear(3, 0);
  }else if ( gpioIntStatus(3, 1) ){
	  encoderDrive( ENCODER_0 );
	  gpioIntClear(3, 1);
  }else if ( gpioIntStatus(3, 2) ){
	  encoderDrive( ENCODER_1 );
	  gpioIntClear(3, 2);
  }else if ( gpioIntStatus(3, 3) ){
	  encoderDrive( ENCODER_1 );
	  gpioIntClear(3, 3);
  }
  return;
}
/* end encoder code */

// Poll every 10 ms until 1 or more keys are pressed, 
//	fill the buffer & return # of chars collected
int pollUSBCDC(char* usbcdcBuf, int len) {
	int  numBytesToRead, numBytesRead, numAvailByte = 0;
	while(numAvailByte == 0) {
		pause(10);
		CDC_OutBufAvailChar (&numAvailByte);
	}
	numBytesToRead = numAvailByte > len ? len : numAvailByte; 
	numBytesRead = CDC_RdOutBuf (usbcdcBuf, &numBytesToRead);
	//printf("[%d]", numBytesRead);
	return numBytesRead;
}

// Check once to see if 1 or more keys have been pressed, 
//	fill the buffer & return # of chars collected
int checkUSBCDC(char* usbcdcBuf, int len) {
	int  numBytesToRead, numBytesRead, numAvailByte = 0;
	CDC_OutBufAvailChar (&numAvailByte);
	numBytesToRead = numAvailByte > len ? len : numAvailByte; 
	numBytesRead = CDC_RdOutBuf (usbcdcBuf, &numBytesToRead);
	//printf("[%d]", numBytesRead);
	return numBytesRead;
}

/**************************************************************************/
/*
End: Special low-level and driver functions
Begin: Initialization functions
*/
/**************************************************************************/

// Set pin 2.10 (onboard LED) as an output
void initLED() {
	gpioSetDir(2, 10, 1);
}

// TODO: switch servo '2' for servo '3' 
// (they are sortof switch in order on the board)
void initServos() {
  /* Enable the clock for CT16B1 */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_CT32B1);

  /* Configure PIO1.1 as Timer1_32MAT0 Output */
  IOCON_JTAG_TDO_PIO1_1 &= ~IOCON_JTAG_TDO_PIO1_1_FUNC_MASK;
  IOCON_JTAG_TDO_PIO1_1 |= IOCON_JTAG_TDO_PIO1_1_FUNC_CT32B1_MAT0;  
  /* Configure PIO1.2 as Timer1_32MAT1 Output */
  IOCON_JTAG_nTRST_PIO1_2 &= ~IOCON_JTAG_nTRST_PIO1_2_FUNC_MASK;
  IOCON_JTAG_nTRST_PIO1_2 |= IOCON_JTAG_nTRST_PIO1_2_FUNC_CT32B1_MAT1;  
  /* Configure PIO1.3 as Timer1_32MAT2 Output */
  IOCON_SWDIO_PIO1_3 &= ~IOCON_SWDIO_PIO1_3_FUNC_MASK;
  IOCON_SWDIO_PIO1_3 |= IOCON_SWDIO_PIO1_3_FUNC_CT32B1_MAT2;
  
  /* Set period (MR3) to 20ms */
  TMR_TMR32B1MR3 = TIMER32_CCLK_100US * 200;

  // 0: 194, 180: 185, 360: 176
  /* Set Duty Cycle (MR0) to 10% */
  TMR_TMR32B1MR0 = TIMER32_CCLK_100US * 194; // 3
  /* Set Duty Cycle (MR1) to 20% */
  TMR_TMR32B1MR1 = TIMER32_CCLK_100US * 185; // 1 
  /* Set Duty Cycle (MR2) to 30% */
  TMR_TMR32B1MR2 = TIMER32_CCLK_100US * 176; // 2

  /* Configure match control register to reset on MR3 */
  TMR_TMR32B1MCR = TMR_TMR32B1MCR_MR3_RESET_ENABLED;

  /* External Match Register Settings for PWM */
  TMR_TMR32B1EMR  = TMR_TMR32B1EMR_EMC0_TOGGLE | TMR_TMR32B1EMR_EM0  |  TMR_TMR32B1EMR_EMC1_TOGGLE | TMR_TMR32B1EMR_EM1 | TMR_TMR32B1EMR_EMC2_TOGGLE | TMR_TMR32B1EMR_EM2;

  /* Enable PWM0 - PWM3 */
  TMR_TMR32B1PWMC = TMR_TMR32B1PWMC_PWM0_ENABLED | TMR_TMR32B1PWMC_PWM1_ENABLED | TMR_TMR32B1PWMC_PWM2_ENABLED | TMR_TMR32B1PWMC_PWM3_ENABLED;

  /* Enable Timer1 */
  TMR_TMR32B1TCR = TMR_TMR32B1TCR_COUNTERENABLE_ENABLED;
}

void initLineSensor(void) {
	i2cInit(I2CMASTER);
}

void initIR(void) {
	adcInit();
}

// backwards1 to 1 to reverse motor one's direction
// backwards2 to 1 to reverse motor two's direction
// (software fix for the case that the wires were plugged in backwards)
void initMotors(int backwards1, int backwards2) {
	/* Enable the clock for CT32B0 */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_CT32B0);

  /* Configure PIO1.1 as Timer1_32MAT0 Output */
  IOCON_PIO1_6 &= ~IOCON_PIO1_6_FUNC_MASK;
  IOCON_PIO1_6 |= IOCON_PIO1_6_FUNC_CT32B0_MAT0;  
  /* Configure PIO1.2 as Timer1_32MAT0 Output */
  IOCON_PIO1_7 &= ~IOCON_PIO1_7_FUNC_MASK;
  IOCON_PIO1_7 |= IOCON_PIO1_7_FUNC_CT32B0_MAT1;  
  
  /* Set period (MR3) to 50us */
  TMR_TMR32B0MR2 = TIMER32_CCLK_1US * 50;

  /* Set Duty Cycle (MR0) to 50% */
  TMR_TMR32B0MR0 = TIMER32_CCLK_1US * 25;
  /* Set Duty Cycle (MR2) to 50% */
  TMR_TMR32B0MR1 = TIMER32_CCLK_1US * 25;

  /* Configure match control register to reset on MR3 */
  TMR_TMR32B0MCR = TMR_TMR32B1MCR_MR2_RESET_ENABLED;

  /* External Match Register Settings for PWM */
  TMR_TMR32B0EMR = TMR_TMR32B0EMR_EMC0_TOGGLE | TMR_TMR32B0EMR_EM0  |  TMR_TMR32B0EMR_EMC1_TOGGLE | TMR_TMR32B0EMR_EM1;

  /* Enable PWM0 - PWM2 */
  TMR_TMR32B0PWMC = TMR_TMR32B0PWMC_PWM0_ENABLED | TMR_TMR32B0PWMC_PWM1_ENABLED | TMR_TMR32B0PWMC_PWM2_ENABLED;

  /* Enable Timer1 */
  TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_ENABLED;
  
  motor1_reversed = backwards1;
  motor2_reversed = backwards2;
}

// backwards1 to 1 to reverse motor one's direction
// backwards2 to 1 to reverse motor two's direction
// (software fix for the case that the wires were plugged in backwards)
void initEncoders(int backwards1, int backwards2) {
	/*written as psedocode until actual values can be set*/
	/* Configure Encoder Pins 3 block (4 pins) as input */

	enc_state[0] = s_0 ;
	enc_state[1] = s_0 ;
	enc_0 = 0;
	enc_1 = 0;

	//gpioInit();	// Already initialized in cpuInit() (yeah, strange.)
	gpioSetDir(3, 0, gpioDirection_Input);
	gpioSetDir(3, 1, gpioDirection_Input);
	gpioSetDir(3, 2, gpioDirection_Input);
	gpioSetDir(3, 3, gpioDirection_Input);

	// Disable Internal pullups
	gpioSetPullup(&IOCON_PIO3_0, gpioPullupMode_Inactive);
	gpioSetPullup(&IOCON_PIO3_1, gpioPullupMode_Inactive);
	gpioSetPullup(&IOCON_PIO3_2, gpioPullupMode_Inactive);
	gpioSetPullup(&IOCON_PIO3_3, gpioPullupMode_Inactive);

	// Enable interupts on A pins
	gpioSetInterrupt(3, 0, gpioInterruptSense_Edge, gpioInterruptEdge_Double, gpioInterruptEvent_ActiveHigh);
	gpioSetInterrupt(3, 1, gpioInterruptSense_Edge, gpioInterruptEdge_Double, gpioInterruptEvent_ActiveHigh);
	gpioSetInterrupt(3, 2, gpioInterruptSense_Edge, gpioInterruptEdge_Double, gpioInterruptEvent_ActiveHigh);
	gpioSetInterrupt(3, 3, gpioInterruptSense_Edge, gpioInterruptEdge_Double, gpioInterruptEvent_ActiveHigh);

	// Enable Interrupts
	gpioIntEnable(3, 0);
	gpioIntEnable(3, 1);	
	gpioIntEnable(3, 2);
	gpioIntEnable(3, 3);
	
	encoder1_reversed = backwards1;
	encoder2_reversed = backwards2;
}

/**************************************************************************/
/*
End: Initialization functions
Begin: Useful higher-level functions
*/
/**************************************************************************/

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

// Poll keyboard until 'enter' is pressed, 
//	mirror chars to screen,
//	fill the buffer & return # of chars collected
int getString(char* buf, int len) {
	char tmpBuf[32];
	char ch = 0;
	int j, numBytesRead = 0;
	for(j = 0; j < 32; j++) tmpBuf[j]=0;
	while(ch != 13) {
		//printf(tmpBuf);
		int numBytesPolled = pollUSBCDC(tmpBuf, 32);
		for(j = 0; j < numBytesPolled; j++)
			buf[numBytesRead++] = tmpBuf[j];
		ch = buf[numBytesRead-1];
		// deal with backspace:
		if (ch == 127) numBytesRead=(numBytesRead-2<0)?0:numBytesRead-2; 
	}
	printf("\r\n");
	// delete '\r' character:
	buf[numBytesRead-1] = 0;
	return numBytesRead-1;
}

// Spin until some key is pressed
void waitForKey() {
	char str[1];
	pollUSBCDC(str, 1);
}

// Get a single character (without waiting for enter)
char getKey() {
	char str[1];
	pollUSBCDC(str, 1);
	return str[0];
}

// Check to see if a key has been pressend 
// if a key hasn't been pressed, returns the null character. 
// Otherwise it will return the first key in the buffer. 
// (it will NOT wait for a key to be pressed before returning)
char checkKey() {
	char str[10];
	int len = checkUSBCDC(str, 10);
	if (len == 0)
		return 0;
	else
		return str[0];
}

// Sets a particular servo to a position based on [degree] 
void moveServo(int servo, int degree) {
	degree = degree%180;
	if (degree < 0)
		degree = degree+180;
	int period = degree+1760;

	if (servo == 1) 
		TMR_TMR32B1MR1 = TIMER32_CCLK_10US * period;
	else if (servo == 2) 
		TMR_TMR32B1MR0 = TIMER32_CCLK_10US * period;
	else if (servo == 3) 
		TMR_TMR32B1MR2 = TIMER32_CCLK_10US * period; 
	else {
		printf("wrong servo%d", servo);
		while(1);
	}
}

// Grabs line sensor data using i2c and dumps into LineSensor array. 
// Returns 1 if successful, 0 if encountered a timeout error 
//	(timeout probably means the line sensor isn't connected correctly)
int readLineSensor(int LineSensor[8]) {
	int i;
	char cmd;

	gpioSetValue(2, 10, 1);
	for(i=0; i<8; i++) //clear i2c buffers
	{
		I2CMasterBuffer[i] = 0;
		I2CSlaveBuffer[i] = 0;
	}

	cmd = 0x84; //1 CH# 01 XX for request conversion. e.g 1 000 01 00 is for channel 2

	for(i=0; i<8; i++)
	{
		////printf("Reading channel %d\n\r", i);
		I2CWriteLength = 2;
		I2CReadLength = 1;
		I2CMasterBuffer[0] = ADS7830_ADDR;
		I2CMasterBuffer[1] = cmd; 
		cmd += 0x10; //increment channel
		I2CMasterBuffer[2] = ADS7830_ADDR | READ_BIT; //not included in writelength b/c its a repeated start
		int timeout = i2cEngine(); //run the transaction and waits for value to be read back
		if (timeout > MAX_TIMEOUT) {
			//printf("ERROR: timeout (device not connected correctly)\n\r");
			return 0;
		}
		LineSensor[i] = I2CSlaveBuffer[0] > 100 ? 999 : 0; //sadly after each transaction the RdIndex is returned to 0, could change i2c driver...
	} 

	return 1;
}

// Gets the value at the ADC requested channel
int getIRValue(int channel) {
	int indexes[] = {0, 1, 6, 7};
	
	if (channel < 0 || channel > 4) {
		printf("wrong IR channel%d", channel);
		while(1);
	}
	
	return (int) adcRead(indexes[channel]);
}

// Set motor 0 or 1 to a duty cycle from 0-100 (0 us to 50 us)
void setMotorDuty(int motor, int power) {
	if (motor == 0) {
		if (motor1_reversed) power = 100-power;
		TMR_TMR32B0MR0 = TIMER32_CCLK_1US / 2 * power;
	} else if (motor == 1) {
		if (motor2_reversed) power = 100-power;
		TMR_TMR32B0MR1 = TIMER32_CCLK_1US / 2 * power;
	} else {
		printf("wrong motor #%d", motor);
		while(1);
	}
}

// Get encoder value corresponding to given encoder
signed long getEncoderValue(encoder_t enc) {	
	if (enc == ENCODER_0) {
		if (encoder1_reversed) return -enc_0;
		return enc_0;
	} else if (enc == ENCODER_1) {
		if (encoder2_reversed) return -enc_1;
		return enc_1;
	}
	printf("wrong encorder #%d", enc);
	while(1);
}

/**************************************************************************/
/*
End: Useful functions
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
		printf("Servo %d %s", servoNum, prompt);
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

void motorDemo() {
	int motorNum = 0, quit = 0;

	printf("\n\rWelcome to the motor demo!\n\r\
Choose an option:\n\rspace - stops motors\n\r\
# - change motor (default is 1)\n\ra - increase duty cycle\n\r\
d - decrease duty cycle\n\rq - quit\n\r");

	while (quit != 1) {
		printf("Motor %d %s", motorNum+1, prompt);
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
			count, lineSensorArr[0], lineSensorArr[4], 
			lineSensorArr[1], lineSensorArr[5], lineSensorArr[2], 
			lineSensorArr[6], lineSensorArr[3], lineSensorArr[7]);
			
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

/**************************************************************************/
/*
End: Demo functions
*/
/**************************************************************************/
