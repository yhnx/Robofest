/*
 * motors.c
 */

#include "motors.h"


float limitPWM(float pwm) {
	if (pwm > PWM_MAX)
		return PWM_MAX;
	else if (pwm < -PWM_MAX)
		return -PWM_MAX;
	return pwm;
}

/*
 * NOTE: your timers and/or timer channels might be different based on what you used when designing your PCB!
 */

void setMotorLPWM(float pwm) {
	if (pwm >= 0) {
		TIM4->CCR4 = 0;
		TIM4->CCR3 = (uint32_t) (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
	else if (pwm < 0) {
		TIM4->CCR3 = 0;
		TIM4->CCR4 = (uint32_t) (-1 * limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
}

void setMotorRPWM(float pwm) {
	if (pwm >= 0) {
		TIM4->CCR2 = 0;
		TIM4->CCR1 = (uint32_t) (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
	else if (pwm < 0) {
		TIM4->CCR1 = 0;
		TIM4->CCR2 = (uint32_t) (-1 * limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
}

void resetMotors() {
	setMotorLPWM(0);
	setMotorRPWM(0);
}