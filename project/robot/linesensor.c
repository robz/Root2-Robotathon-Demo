#include "project/robot/linesensor.h"
#include "core/i2c/i2c.h"
#include "sysinit.h"

//leftshifting 1 for space for R/W bit
#define ADS7830_ADDR (0x48 << 1)
#define READ_BIT 1

void initLineSensor(void) {
	i2cInit(I2CMASTER);
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
		LineSensor[i] = I2CSlaveBuffer[0]; //> 100 ? 999 : 0; //sadly after each transaction the RdIndex is returned to 0, could change i2c driver...
	} 

	return 1;
}