#include "project/robot/IR.h"
#include "sysinit.h"
#include "core/adc/adc.h"

void initIR(void) {
	adcInit();
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