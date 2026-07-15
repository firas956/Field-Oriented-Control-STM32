#include "core/hall_pll.h"
#include "hw/hw_hall_sensor.h"
#include <math.h>

#define TWO_PI 6.28318530718f
#define PI     3.14159265359f

static HallPLL_t internal_pll;
static uint8_t pll_initialized = 0;

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
    
    // Anti-windup (optional but highly recommended, adjust limits to your motor's max rad/s)
    // if (pll->integral_term > MAX_SPEED) pll->integral_term = MAX_SPEED;
    // if (pll->integral_term < -MAX_SPEED) pll->integral_term = -MAX_SPEED;

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

float HW_Hall_GetElectricalAngle(void) {
    // Lazy initialization so you don't even have to modify MotorControl_Init()
    if (!pll_initialized) {
        // Run at 10kHz (Ts = 0.0001s). Tune Kp and Ki as needed.
        HallPLL_Init(&internal_pll, 100.0f, 4000.0f, 0.00005f);
        pll_initialized = 1;
    }

    // 1. Fetch the raw, coarse angle from the weak hardware layer's background variable
    float coarse_angle = HW_Hall_GetBaseAngle();

    // 2. Step the PLL math forward
    HallPLL_Update(&internal_pll, coarse_angle);

    // 3. Return the beautifully smoothed PLL angle to the FOC loop
    return internal_pll.est_angle;
}

// Strong override for the speed getter
float HW_Hall_GetSpeedRPM(uint8_t pole_pairs) {
    // Convert the PLL's estimated electrical speed (rad/s) to mechanical RPM
    float omega_elec = internal_pll.est_speed;
    float rpm = (omega_elec * 60.0f) / (2.0f * 3.14159265359f * (float)pole_pairs);
    return rpm;
}