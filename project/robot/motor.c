#include "project/robot/motor.h"
#include "sysinit.h"
#include "core/timer32/timer32.h"

int motor1_reversed = 0, motor2_reversed = 0;

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