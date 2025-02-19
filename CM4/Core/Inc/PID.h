/*
 * PID.h
 *
 *  Created on: Nov 15, 2024
 *      Author: giovannimasi
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <main.h>

#define KPP 0.016
#define KIP 0.0002
#define KDP 0.06
#define KPR 0.016
#define KIR 0.0001
#define KDR 0.06

/*double KPP = 0.04;
double KIP = 0.0000;
double KDP = 0.05;
double KPR = 0.04;
double KIR = 0.0000;
double KDR = 0.04;*/

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
