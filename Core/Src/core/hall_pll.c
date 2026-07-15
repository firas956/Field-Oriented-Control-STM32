#include "core/hall_pll.h"

#define TWO_PI 6.28318530718f
#define PI     3.14159265359f

// Anti-windup limit for the PLL integrator (electrical rad/s).
// 4000 rad/s el = ~19000 mech RPM at 2 pole pairs: far above anything
// this motor can do, so it only catches divergence, not normal operation.
#define PLL_MAX_SPEED_RAD_S 4000.0f

void HallPLL_Init(HallPLL_t *pll, float kp, float ki, float dt) {
    pll->kp = kp;
    pll->ki = ki;
    pll->dt = dt;
    pll->integral_term = 0.0f;
    pll->est_angle = 0.0f;
    pll->est_speed = 0.0f;
}

void HallPLL_Update(HallPLL_t *pll, float measured_angle) {
    // 1. Calculate phase error
    float error = measured_angle - pll->est_angle;

    // 2. Shortest path wrap-around handling for the error [-PI, PI]
    while (error > PI)  error -= TWO_PI;
    while (error < -PI) error += TWO_PI;

    // 3. PI Controller to calculate estimated speed
    pll->integral_term += pll->ki * error * pll->dt;

    // Anti-windup on the integrator
    if (pll->integral_term > PLL_MAX_SPEED_RAD_S)  pll->integral_term = PLL_MAX_SPEED_RAD_S;
    if (pll->integral_term < -PLL_MAX_SPEED_RAD_S) pll->integral_term = -PLL_MAX_SPEED_RAD_S;

    pll->est_speed = (pll->kp * error) + pll->integral_term;

    // 4. Integrate speed to get estimated angle
    pll->est_angle += pll->est_speed * pll->dt;

    // 5. Wrap estimated angle to [0, 2*PI]
    while (pll->est_angle >= TWO_PI) pll->est_angle -= TWO_PI;
    while (pll->est_angle < 0.0f)    pll->est_angle += TWO_PI;
}

void HallPLL_Reset(HallPLL_t *pll, float current_angle) {
    pll->integral_term = 0.0f;
    pll->est_speed = 0.0f;
    pll->est_angle = current_angle;
}
