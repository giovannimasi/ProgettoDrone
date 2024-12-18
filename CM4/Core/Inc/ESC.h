/*
 * ESC.h
 *
 *  Created on: Dec 13, 2024
 *      Author: giovannimasi
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

#include "main.h"

#define OFF_DUTY 5
#define ARMING_DUTY 5.3
#define MIN_DUTY 5.4
#define MAX_DUTY 6.5
#define LIMIT_DUTY 10
#define MAX_SPEED 11830
#define MIN_SPEED 2000
#define b 0.00001163
#define d 0.00008

void ESC_Calibrate();
float rangeDuty(float duty);
void setPWM(float pwm1, float pwm2, float pwm3, float pwm4);
float* SpeedCompute(float virtualInputs[]);
float map(float val);

#endif /* INC_ESC_H_ */
