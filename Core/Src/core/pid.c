#include "core/pid.h"

void PI_Init(PI_Controller_t *pid, float Kp, float Ki, float ts, float out_min, float out_max){

    pid->Kp = Kp;
    pid->Ki_normalized = Ki*ts;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integrator = 0.0f;
}
void PI_Reset(PI_Controller_t *pid, float error){

    pid->integrator = 0.0f;
}
float PI_Update(PI_Controller_t *pid, float error){
    float p_term = pid->Kp * error;
    pid->integrator += pid->Ki_normalized *error;
    
    //anti-windup
    if (pid->integrator > pid->out_max){
        pid->integrator = pid->out_max;
    }
    else if (pid->integrator < pid->out_min){
        pid->integrator = pid->out_min;
    }

    float output = p_term + pid->integrator;

    // The integrator clamp alone lets P push the sum past the limits
    if (output > pid->out_max){
        output = pid->out_max;
    }
    else if (output < pid->out_min){
        output = pid->out_min;
    }

    return output;
}