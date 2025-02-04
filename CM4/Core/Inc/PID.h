/*
 * PID.h
 *
 *  Created on: Nov 15, 2024
 *      Author: giovannimasi
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <main.h>

#define KPP 0.0139
#define KIP 0
#define KDP 0
#define KPR 0.016
#define KIR 0
#define KDR 0

typedef struct PID{
    // TUNING PARAM (parallel form)
    float Kp;			// P gain
    float Ki;			// I gain
    float Kd;			// D gain

    // OTHER PARAM
    float dt;			//period
    float Iterm;
    float lastError;
    float u_max;  		// PID output upper limit
    float u_min;  		// PID output lower limit
} PID;

// Function declaration
void init_PID(PID* p, float Kp, float Ki, float Kd, float dt, float u_max, float u_min);
float PID_controller(PID* p, float input, float setPoint);

#endif /* INC_PID_H_ */
