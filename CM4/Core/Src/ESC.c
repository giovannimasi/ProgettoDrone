/*
 * ESC.c
 *
 *  Created on: Dec 13, 2024
 *      Author: giovannimasi
 */
#include "main.h"
#include <ESC.h>


void ESC_Calibrate(){
	TIM3->CCR1 = (uint32_t) (TIM3->ARR * LIMIT_DUTY / 100);
	TIM3->CCR2 = (uint32_t) (TIM3->ARR * LIMIT_DUTY / 100);
	TIM3->CCR3 = (uint32_t) (TIM3->ARR * LIMIT_DUTY / 100);
	TIM3->CCR4 = (uint32_t) (TIM3->ARR * LIMIT_DUTY / 100);
	HAL_Delay(3000);
	TIM3->CCR1 = (uint32_t) (TIM3->ARR * OFF_DUTY / 100);
	TIM3->CCR2 = (uint32_t) (TIM3->ARR * OFF_DUTY / 100);
	TIM3->CCR3 = (uint32_t) (TIM3->ARR * OFF_DUTY / 100);
	TIM3->CCR4 = (uint32_t) (TIM3->ARR * OFF_DUTY / 100);
}

float rangeDuty(float duty){
	if(duty<MIN_DUTY){
		return MIN_DUTY;
	}
	else if (duty>MAX_DUTY){
		return MAX_DUTY;
	}
	return duty;
}

void setPWM(float pwm1, float pwm2, float pwm3, float pwm4){
	TIM3->CCR1 = (uint32_t) (TIM3->ARR * rangeDuty(pwm1) / 100);
	TIM3->CCR2 = (uint32_t) (TIM3->ARR * rangeDuty(pwm2) / 100);
	TIM3->CCR3 = (uint32_t) (TIM3->ARR * rangeDuty(pwm3) / 100);
	TIM3->CCR4 = (uint32_t) (TIM3->ARR * rangeDuty(pwm4) / 100);
}

float* SpeedCompute(float virtualInputs[])
{
    static float Speeds_quad[4];
    static float Speeds[4];

    Speeds_quad[0] = (1/(4*b))*virtualInputs[0] 							   + (1/(2*l*b))*virtualInputs[2] - (1/(4*d))*virtualInputs[3];
    Speeds_quad[1] = (1/(4*b))*virtualInputs[0] - (1/(2*l*b))*virtualInputs[1] 								  + (1/(4*d))*virtualInputs[3];
    Speeds_quad[2] = (1/(4*b))*virtualInputs[0] 							   - (1/(2*l*b))*virtualInputs[2] - (1/(4*d))*virtualInputs[3];
    Speeds_quad[3] = (1/(4*b))*virtualInputs[0] + (1/(2*l*b))*virtualInputs[1] 								  + (1/(4*d))*virtualInputs[3];

    /*
     * Calcoliamo le velocità dei motori al quadrato, poiché non possono essere negative.
     * Partendo dal valore di throttle e seguendo le matrici di controllo dei droni andiamo
     * a sommare e sottrarre le variabili date tramite il PID per il controllo delle velocità.
     */

    Speeds[0] = sqrt(Speeds_quad[0]);
    Speeds[1] = sqrt(Speeds_quad[1]);
    Speeds[2] = sqrt(Speeds_quad[2]);
    Speeds[3] = sqrt(Speeds_quad[3]);

    // Una volta calcolata la velocità dei motori al quadrato, viene eseguita la radice

    return Speeds;
}


float map(float val){
	float duty = (((MAX_DUTY-MIN_DUTY)*val) + ((MIN_DUTY * MAX_SPEED)-(MAX_DUTY*MIN_SPEED)))/(MAX_SPEED-MIN_SPEED);

	if (duty<MIN_DUTY) duty=MIN_DUTY;
	else if (duty>MAX_DUTY) duty = MAX_DUTY;
	return duty;
}



