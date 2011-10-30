#include "project/robotathon_drivers/servo.h"
#include "sysinit.h"
#include "core/timer32/timer32.h"

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