#ifndef PID_H
#define PID_H

typedef struct {

    float Kp;
    float Ki_normalized;
    float out_max;
    float out_min;
    float integrator;
} PI_Controller_t;

    void PI_Init(PI_Controller_t *pid, float Kp, float Ki, float ts, float out_min, float out_max);
    void PI_Reset(PI_Controller_t *pid, float error);
    float PI_Update(PI_Controller_t *pid, float error);

#endif