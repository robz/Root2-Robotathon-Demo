#include "project/robotathon_drivers/encoder.h"
#include "sysinit.h"

/* encoder code */
/* Note: will need to add support for directions (software reverse) later. Same for motor code.*/
/* Move typedefs into header when sorting code 
 * Also recommending same typedef style for all other aspects of this project */

typedef enum { s_0, s_1, s_2, s_3 }  encoder_state_t;
const int enc_Machine[4][4] = { { 0 , 1 , -1 , 0 } , {-1 , 0 , 0 , 1 } , { 1 , 0 , 0 , -1 } , { 0 , -1 , 1 , 0 } }; 
volatile static encoder_count_t enc_0, enc_1;
volatile static encoder_state_t enc_state[2];

int encoder1_reversed = 0, 
	encoder2_reversed = 0;

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