#include "project/robot/usb.h"
#include "sysinit.h"
#include "core/usbcdc/cdcuser.h"

#define UARTBUFFERSIZE 5

// Poll every 10 ms until 1 or more keys are pressed, 
//	fill the buffer & return # of chars collected
int pollUSBCDC(char* usbcdcBuf, int len) {
	int  numBytesToRead, numBytesRead, numAvailByte = 0;
	while(numAvailByte == 0) {
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
