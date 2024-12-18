#include <PID.h>

void init_PID(PID* p, float Kp, float Ki, float Kd, float dt, float u_max, float u_min){
	p->Kp=Kp;
	p->Ki=Ki;
	p->Kd=Kd;
	p->dt = dt;
	p->Iterm = 0;
	p->lastError = 0;
	p->u_max=u_max;
	p->u_min=u_min;
}

float PID_controller(PID* p, float input, float setPoint){
    float u;
    float newIterm;

    float e = setPoint - input;

    float Pterm = p->Kp * e;
    newIterm = p->Iterm + (p->Ki)*p->dt * p->lastError;
    float Dterm = (p->Kd/p->dt) * (e - p->lastError);

    p->lastError = e;

    u = Pterm + newIterm + Dterm;

    if(u > p->u_max){
        u = p->u_max; // upper limit saturation
    } else if (u < p->u_min){
        u = p->u_min; // lower limit saturation
    } else {
        p->Iterm= newIterm; // clamping anti-windup
    }

    return u;
}
